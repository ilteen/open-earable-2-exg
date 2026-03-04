#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <errno.h>
#include "../SensorManager/SensorManager.h"
#include "../ParseInfo/SensorScheme.h"
#include "../../utils/StateIndicator.h"

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_manager, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

#define MAX_SENSOR_REC_NAME_LENGTH 64

static struct k_thread thread_data_notify;

static k_tid_t thread_id_notify;

ZBUS_SUBSCRIBER_DEFINE(sensor_gatt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_CHAN_DECLARE(sensor_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);

static K_THREAD_STACK_DEFINE(thread_stack_notify, CONFIG_SENSOR_GATT_NOTIFY_STACK_SIZE);

K_MSGQ_DEFINE(gatt_queue, sizeof(struct sensor_data), CONFIG_SENSOR_GATT_SUB_QUEUE_SIZE, 4);

//static struct sensor_msg msg;
static struct sensor_data sensor_data;
static struct sensor_config config;

static bool notify_enabled = false;
static bool sensor_config_status_ntfy_enabled = false;

void set_sensor_recording_name(const char *name);
static char sensor_recording_name[MAX_SENSOR_REC_NAME_LENGTH] = "recording_";

static struct sensor_config *active_sensor_configs;
static size_t active_sensor_configs_size = 0;

static void connect_evt_handler(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen2, connect_evt_handler); //static

void sensor_queue_listener_cb(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(sensor_queue_listener, sensor_queue_listener_cb);

static bool connection_complete = false;
static bool scheduled_start_active = false;
static uint64_t scheduled_start_unix_us = 0;
static char scheduled_recording_name[MAX_SENSOR_REC_NAME_LENGTH] = "recording_";

struct __packed scheduled_sensor_start_cfg {
	uint8_t sensorId;
	uint8_t sampleRateIndex;
	uint8_t storageOptions;
	uint8_t reserved;
	uint64_t unixStartTimeUs;
};

#define SCHEDULED_START_MIN_LEAD_TIME_US 1000000ULL
#define SCHEDULED_START_MAX_LEAD_TIME_US (365ULL * 24ULL * 60ULL * 60ULL * 1000000ULL)

// Fixed recording rates used for both immediate and scheduled start.
#define EXG_RECORD_SAMPLE_RATE_INDEX 4U  // ExG = 256 Hz
#define IMU_RECORD_SAMPLE_RATE_INDEX 3U  // IMU = 200 Hz
#define PPG_RECORD_SAMPLE_RATE_INDEX 4U  // PPG (left/right) = 200 Hz
#define TEMP_RECORD_SAMPLE_RATE_INDEX 4U // Temp (left/right) = 8 Hz

static void scheduled_sensor_start_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(scheduled_sensor_start_work, scheduled_sensor_start_work_handler);
static void start_recording_sensor_suite(void);

int notify_count = 0;

int MAX_NOTIFIES_IN_FLIGHT = 4;

static void schedule_or_start_sensor_config(void)
{
	uint64_t now_us = micros();

	if (scheduled_start_unix_us <= now_us) {
		scheduled_start_active = false;
		scheduled_start_unix_us = 0;
		state_indicator_set_sd_state(SD_IDLE);
		/* Use the name captured when schedule was configured. */
		set_sensor_recording_name(scheduled_recording_name);
		start_recording_sensor_suite();
		return;
	}

	uint64_t remaining_us = scheduled_start_unix_us - now_us;
	uint32_t delay_ms = (remaining_us > 1000000ULL) ? 1000U : (uint32_t)((remaining_us + 999ULL) / 1000ULL);
	k_work_schedule(&scheduled_sensor_start_work, K_MSEC(delay_ms));
}

static void config_recording_sensor(uint8_t sensor_id, uint8_t sample_rate_index)
{
	struct sensor_config sensor_cfg = {
		.sensorId = sensor_id,
		.sampleRateIndex = sample_rate_index,
		.storageOptions = DATA_STORAGE,
	};
	config_sensor(&sensor_cfg);
}

static void start_recording_sensor_suite(void)
{
	config_recording_sensor(ID_EXG, EXG_RECORD_SAMPLE_RATE_INDEX);
	config_recording_sensor(ID_IMU, IMU_RECORD_SAMPLE_RATE_INDEX);
	config_recording_sensor(ID_PPG_right_I2C2, PPG_RECORD_SAMPLE_RATE_INDEX);
	config_recording_sensor(ID_PPG_left_I2C3, PPG_RECORD_SAMPLE_RATE_INDEX);
	config_recording_sensor(ID_OPTTEMP_right_I2C2, TEMP_RECORD_SAMPLE_RATE_INDEX);
	config_recording_sensor(ID_OPTTEMP_left_I2C3, TEMP_RECORD_SAMPLE_RATE_INDEX);
}

static void cancel_scheduled_sensor_start(void)
{
	if (!scheduled_start_active) {
		return;
	}

	scheduled_start_active = false;
	scheduled_start_unix_us = 0;
	k_work_cancel_delayable(&scheduled_sensor_start_work);
	state_indicator_set_sd_state(SD_IDLE);
}

static void scheduled_sensor_start_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!scheduled_start_active) {
		return;
	}

	schedule_or_start_sensor_config();
}

static void connect_evt_handler(const struct zbus_channel *chan)
{
	const struct bt_mgmt_msg *msg;

	msg = zbus_chan_const_msg(chan);

	switch (msg->event) {
	case BT_MGMT_CONNECTED:
		connection_complete = true;
		break;

	case BT_MGMT_DISCONNECTED:
		connection_complete = false;
		notify_enabled = false;
		k_msgq_purge(&gatt_queue);
		break;
	}
}

static void sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("Sensor data notifications %s", notify_enabled ? "enabled" : "disabled");

	k_msgq_purge(&gatt_queue);
}

static void sensor_config_status_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	sensor_config_status_ntfy_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static int schedule_sensor_start_internal(uint8_t sensor_id, uint8_t sample_rate_index,
					  uint8_t storage_options, uint64_t start_time_us)
{
	if (sensor_id != ID_EXG) {
		LOG_WRN("Write scheduled start: Unsupported sensor id %u (only EXG is allowed)", sensor_id);
		return -EINVAL;
	}

	if ((storage_options & DATA_STORAGE) == 0) {
		LOG_WRN("Write scheduled start: DATA_STORAGE not set");
		return -EINVAL;
	}

	ARG_UNUSED(sample_rate_index);

	uint64_t now_us = micros();
	if (start_time_us <= (now_us + SCHEDULED_START_MIN_LEAD_TIME_US)) {
		LOG_WRN("Write scheduled start: Start time too early (start=%llu now=%llu)",
			(unsigned long long)start_time_us, (unsigned long long)now_us);
		return -ERANGE;
	}

	if (start_time_us > (now_us + SCHEDULED_START_MAX_LEAD_TIME_US)) {
		LOG_WRN("Write scheduled start: Start time too far in future (start=%llu now=%llu)",
			(unsigned long long)start_time_us, (unsigned long long)now_us);
		return -ERANGE;
	}

	if (strlen(sensor_recording_name) == 0 || strcmp(sensor_recording_name, "recording_") == 0) {
		LOG_WRN("Write scheduled start: Recording name not configured");
		return -EINVAL;
	}

	/* Ensure no sensors keep running while waiting for scheduled start. */
	stop_sensor_manager();
	cancel_scheduled_sensor_start();

	scheduled_start_unix_us = start_time_us;
	strncpy(scheduled_recording_name, sensor_recording_name, sizeof(scheduled_recording_name) - 1);
	scheduled_recording_name[sizeof(scheduled_recording_name) - 1] = '\0';
	scheduled_start_active = true;

	state_indicator_set_sd_state(SD_WAITING_SCHEDULED_START);
	schedule_or_start_sensor_config();

	LOG_INF("Scheduled EXG start at %llu us (now=%llu, in %llu us, sr_idx=%u)",
		(unsigned long long)scheduled_start_unix_us,
		(unsigned long long)now_us,
		(unsigned long long)(scheduled_start_unix_us - now_us),
		EXG_RECORD_SAMPLE_RATE_INDEX);

	return 0;
}

static ssize_t write_config(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != sizeof(struct sensor_config)) {
		LOG_WRN("Write sensor config: Incorrect data length: Expected %i but got %i", sizeof(struct sensor_config), len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_WRN("Write sensor config: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	struct sensor_config * config = (struct sensor_config *)buf;
	cancel_scheduled_sensor_start();

	if (config->storageOptions == 0) {
		LOG_INF("Setup sensor ID %i (turned off)", config->sensorId);
	} else {
		LOG_INF("Setup sensor ID %i with samplerateIndex %i", config->sensorId, config->sampleRateIndex);
	}

	/* EXG storage write from the app means "start recording now" with full sensor set. */
	if (config->sensorId == ID_EXG && (config->storageOptions & DATA_STORAGE) != 0U) {
		start_recording_sensor_suite();
	} else {
		config_sensor((struct sensor_config *) buf);
	}

	return len;
}

static ssize_t write_scheduled_start(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0) {
		LOG_WRN("Write scheduled start: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(struct scheduled_sensor_start_cfg) && len != sizeof(uint64_t)) {
		LOG_WRN("Write scheduled start: Incorrect data length: Expected %i or %i but got %i",
			(int)sizeof(struct scheduled_sensor_start_cfg), (int)sizeof(uint64_t), len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *p = (const uint8_t *)buf;
	uint8_t sensor_id = ID_EXG;
	uint8_t sample_rate_index = EXG_RECORD_SAMPLE_RATE_INDEX;
	uint8_t storage_options = DATA_STORAGE;
	uint64_t start_time_us;

	if (len == sizeof(struct scheduled_sensor_start_cfg)) {
		sensor_id = p[0];
		sample_rate_index = p[1];
		storage_options = p[2];
		start_time_us = sys_get_le64(&p[4]);
	} else {
		/* Backward-compatible mode: payload only contains unix start time (u64 LE). */
		start_time_us = sys_get_le64(p);
	}

	int ret = schedule_sensor_start_internal(sensor_id, sample_rate_index, storage_options, start_time_us);
	if (ret != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	return len;
}

static ssize_t read_sensor_rec_name(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const char *name = get_sensor_recording_name();
	size_t name_len = strlen(name);

	if (offset > name_len) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, name, name_len);
}

static ssize_t write_sensor_rec_name(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write rec-name, len: %u, offset: %u, flags: 0x%02x, handle: %u, conn: %p",
		len, offset, flags, attr->handle, (void *)conn);

	if (offset >= (MAX_SENSOR_REC_NAME_LENGTH - 1)) {
		LOG_WRN("Write sensor recording name: Invalid offset %u", offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if ((offset + len) > (MAX_SENSOR_REC_NAME_LENGTH - 1)) {
		LOG_WRN("Write sensor recording name: Data too long (offset=%u len=%u, max=%u)",
			offset, len, (unsigned int)(MAX_SENSOR_REC_NAME_LENGTH - 1));
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	/* For prepare writes, only validate; commit happens on execute. */
	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return len;
	}

	if (offset == 0U) {
		sensor_recording_name[0] = '\0';
	}

	memcpy(&sensor_recording_name[offset], buf, len);
	sensor_recording_name[offset + len] = '\0';

	LOG_DBG("Write sensor recording name committed: %s", sensor_recording_name);

	return len;
}

static ssize_t read_sensor_config_status(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const uint16_t size = sizeof(struct sensor_config) * active_sensor_configs_size;
	LOG_DBG("Reading sensor config status");

	if (len < size) {
		LOG_WRN("Read sensor config status: Buffer too small: %u < %u", len, size);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, active_sensor_configs, size);
}

BT_GATT_SERVICE_DEFINE(sensor_service,
BT_GATT_PRIMARY_SERVICE(BT_UUID_SENSOR),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_CONFIG,
            BT_GATT_CHRC_WRITE,
            BT_GATT_PERM_WRITE,
            NULL, write_config, &config),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA,
			BT_GATT_CHRC_NOTIFY,
			BT_GATT_PERM_NONE,
			NULL, NULL, &sensor_data),
BT_GATT_CCC(sensor_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_CONFIG_STATUS,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			BT_GATT_PERM_READ,
			read_sensor_config_status, NULL, &active_sensor_configs),
BT_GATT_CCC(sensor_config_status_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_RECORDING_NAME,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			read_sensor_rec_name, write_sensor_rec_name, NULL),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_SCHEDULED_START,
			BT_GATT_CHRC_WRITE,
			BT_GATT_PERM_WRITE,
			NULL, write_scheduled_start, NULL),
);

static void notify_complete() {
	notify_count--;

	if (notify_count < 0) {
		notify_count = 0;
		LOG_WRN("Notify count went below zero!");
	}
}

static void notification_task(void) {
	int ret;

	while (1) {
		ret = k_msgq_get(&gatt_queue, &sensor_data, K_FOREVER);

		if (ret != 0) {
			LOG_WRN("No data to process");
			continue;
		}

		if (connection_complete && notify_enabled) {
			const uint16_t size = sizeof(sensor_data.id) + sizeof(sensor_data.size) + sizeof(sensor_data.time) + sensor_data.size;

			static struct bt_gatt_notify_params params;
			params.attr = &sensor_service.attrs[4];
			params.data = &sensor_data;
			params.len = size;
			params.func = notify_complete;
			params.user_data = NULL;

			while(notify_count >= MAX_NOTIFIES_IN_FLIGHT) {
				k_yield(); // maybe replace with k_sleep?
			}

			notify_count++;

			ret = bt_gatt_notify_cb(NULL, &params);
			if (ret != 0) {
				LOG_WRN("Failed to send data: %d.\n", ret);
			}
		}
	}
}

void sensor_queue_listener_cb(const struct zbus_channel *chan) {
	int ret;
	const struct sensor_msg * msg;

    msg = (struct sensor_msg *)zbus_chan_const_msg(&sensor_chan);

	if (msg->stream) {
		ret = k_msgq_put(&gatt_queue, &msg->data, K_NO_WAIT);

		if (ret) {
			LOG_WRN("ble sensor stream queue full");
		}
	}
}

int init_sensor_config_status() {
	struct ParseInfoScheme *parse_info_scheme = getParseInfoScheme();

	// Initialize the active sensor configs list
	active_sensor_configs_size = parse_info_scheme->sensorCount;
	active_sensor_configs = k_malloc(sizeof(struct sensor_config) * active_sensor_configs_size);
	if (active_sensor_configs == NULL) {
		LOG_ERR("Failed to allocate memory for active sensor configs");
		return -1;
	}

	for (size_t i = 0; i < active_sensor_configs_size; i++) {
		struct SensorScheme *sensor_scheme = getSensorSchemeForId(parse_info_scheme->sensorIds[i]);
		LOG_DBG("Initializing sensor config state for sensor with id %d", sensor_scheme->id);

		active_sensor_configs[i].sensorId = sensor_scheme->id;
		if (sensor_scheme->configOptions.availableOptions & FREQUENCIES_DEFINED) {
			active_sensor_configs[i].sampleRateIndex = sensor_scheme->configOptions.frequencyOptions.defaultFrequencyIndex;
		} else {
			active_sensor_configs[i].sampleRateIndex = 0; // Default to 0 if frequencies are not defined
		}
		active_sensor_configs[i].storageOptions = 0; // Default storage options
	}

	LOG_DBG("Sensor config status initialized");
	return 0;
}

int set_sensor_config_status(struct sensor_config config) {
	LOG_DBG("Setting sensor config status for sensorId: %i", config.sensorId);

	ssize_t sensor_config_index = -1;
	for (size_t i = 0; i < active_sensor_configs_size; i++) {
		if (active_sensor_configs[i].sensorId == config.sensorId) {
			sensor_config_index = i;
			break;
		}
	}

	if (sensor_config_index >= 0) {
		active_sensor_configs[sensor_config_index] = config;
		LOG_DBG("Found sensor config");
	} else {
		LOG_DBG("Sensor config not found, adding new sensor config");
		// allocate more space for the new sensor config list
		active_sensor_configs_size++;
		struct sensor_config *new_active_sensor_configs = k_realloc(active_sensor_configs, active_sensor_configs_size);
		if (new_active_sensor_configs == NULL) {
			LOG_ERR("Failed to allocate memory for new sensor config");
			return -1;
		}
		active_sensor_configs = new_active_sensor_configs;
		active_sensor_configs[active_sensor_configs_size - 1] = config;
	}

	if (sensor_config_status_ntfy_enabled) {
		LOG_DBG("Sensor config status notification, notifying %zu active sensor configs", active_sensor_configs_size);
		struct bt_gatt_notify_params params = {
            .attr   = &sensor_service.attrs[7],
            .data   = active_sensor_configs,
            .len    = sizeof(struct sensor_config) * active_sensor_configs_size,
        };
        int ret = bt_gatt_notify_cb(NULL, &params);

		if (ret) {
			LOG_ERR("Failed to notify sensor config status, error code: %d", ret);
			return ret;
		}
	}

	return 0;
}

int init_sensor_service() {
	int ret;

	thread_id_notify = k_thread_create(
		&thread_data_notify, thread_stack_notify,
		CONFIG_SENSOR_GATT_NOTIFY_STACK_SIZE, (k_thread_entry_t)notification_task, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_SENSOR_GATT_NOTIFY_THREAD_PRIO), 0, K_NO_WAIT);

	ret = k_thread_name_set(thread_id_notify, "SENSOR_GATT_NOTIFY");
	if (ret) {
		LOG_ERR("Failed to create sensor_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&sensor_chan, &sensor_queue_listener, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add sensor sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&bt_mgmt_chan, &bt_mgmt_evt_listen2, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add bt_mgmt listener");
		return ret;
	}

	init_sensor_config_status();

    return 0;
}

const char *get_sensor_recording_name() {
	return sensor_recording_name;
}

/**
 * @brief Set the sensor recording name object.
 *
 * @param name A pointer to the name string.
 * Has to be a valid string with a length greater than 0
 * and 0 terminated.
 */
void set_sensor_recording_name(const char *name) {
	if (name == NULL || strlen(name) == 0) {
		LOG_WRN("Invalid sensor recording name");
		return;
	}

	strncpy(sensor_recording_name, name, sizeof(sensor_recording_name) - 1);
	sensor_recording_name[sizeof(sensor_recording_name) - 1] = '\0';
}

int sensor_service_set_recording_name(const char *name)
{
	if (name == NULL || strlen(name) == 0) {
		return -EINVAL;
	}

	set_sensor_recording_name(name);
	return 0;
}

int sensor_service_schedule_start(uint8_t sensor_id, uint8_t sample_rate_index, uint8_t storage_options,
				  uint64_t start_time_us)
{
	return schedule_sensor_start_internal(sensor_id, sample_rate_index, storage_options, start_time_us);
}

int sensor_service_schedule_exg_start(uint8_t sample_rate_index, uint64_t start_time_us)
{
	ARG_UNUSED(sample_rate_index);
	return schedule_sensor_start_internal(ID_EXG, EXG_RECORD_SAMPLE_RATE_INDEX, DATA_STORAGE, start_time_us);
}
