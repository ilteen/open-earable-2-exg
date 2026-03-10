#include "time_sync.h"
#include "usb_time_sync.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/devicetree.h>
#include <zephyr/settings/settings.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <limits.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/toolchain/common.h>

#include "openearable_common.h"
#include "../Battery/BootState.h"

// External reference to sensor work queue (defined in SensorManager.cpp)
extern struct k_work_q sensor_work_q;

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(time_sync, LOG_LEVEL_DBG);

#define BT_UUID_TIME_SYNC_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x2e04cbf7, 0x939d, 0x4be5, 0x823e, 0x271838b75259)
#define BT_UUID_TIME_SYNC_OFFSET_CHARAC_VAL \
    BT_UUID_128_ENCODE(0x2e04cbf8, 0x939d, 0x4be5, 0x823e, 0x271838b75259)
#define BT_UUID_TIME_SYNC_RTT_CHARAC_VAL \
    BT_UUID_128_ENCODE(0x2e04cbf9, 0x939d, 0x4be5, 0x823e, 0x271838b75259)

#define BT_UUID_TIME_SYNC_SERVICE           BT_UUID_DECLARE_128(BT_UUID_TIME_SYNC_SERVICE_VAL)
#define BT_UUID_TIME_SYNC_OFFSET_CHARAC     BT_UUID_DECLARE_128(BT_UUID_TIME_SYNC_OFFSET_CHARAC_VAL)
#define BT_UUID_TIME_SYNC_RTT_CHARAC        BT_UUID_DECLARE_128(BT_UUID_TIME_SYNC_RTT_CHARAC_VAL)

enum time_sync_op {
    TIME_SYNC_OP_REQUEST = 0,
    TIME_SYNC_OP_RESPONSE = 1,
};

struct __packed time_sync_packet {
    uint8_t  version;       // Version of the time sync packet
    uint8_t  op;            // 0 = request, 1 = response
    uint16_t seq;           // Sequence number, phone chooses
    uint64_t t1_phone;      // phone send time
    uint64_t t2_dev_rx;     // device receive time
    uint64_t t3_dev_tx;     // device transmit time
};

static int64_t time_offset_us = 0;
static bool time_synced = false;
static time_sync_callback_t time_sync_callback = NULL;

#define TIME_SYNC_RETAINED_MAGIC 0x5453594EU /* "TSYN" */
#define TIME_SYNC_RETAINED_VERSION 1U
#define TIME_SYNC_MIN_OFFSET_US 1577836800000000LL /* 2020-01-01 */
#define TIME_SYNC_MAX_OFFSET_US 4102444800000000LL /* 2100-01-01 */
#define TIME_SYNC_SETTINGS_KEY "oe_time/offset"
#define TIME_SYNC_SETTINGS_LAST_UNIX_KEY "oe_time/last_unix"
#define TIME_SYNC_SNAPSHOT_PERSIST_INTERVAL_US 2000000ULL

struct __packed retained_time_sync_state {
	uint32_t magic;
	uint16_t version;
	uint16_t reserved;
	int64_t offset_us;
	uint32_t checksum;
};

static struct retained_time_sync_state retained_time_sync __noinit;
static int64_t settings_time_sync_offset_us = 0;
static bool settings_time_sync_offset_valid = false;
static uint64_t settings_last_unix_us = 0;
static bool settings_last_unix_valid = false;
static uint64_t last_snapshot_persist_us = 0;

static uint32_t checksum_fnv1a32(const uint8_t *data, size_t len)
{
	uint32_t hash = 2166136261U;
	for (size_t i = 0; i < len; ++i) {
		hash ^= data[i];
		hash *= 16777619U;
	}
	return hash;
}

static bool time_offset_in_valid_range(int64_t offset_us)
{
	return (offset_us >= TIME_SYNC_MIN_OFFSET_US) && (offset_us <= TIME_SYNC_MAX_OFFSET_US);
}

static uint32_t retained_time_sync_checksum(const struct retained_time_sync_state *state)
{
	return checksum_fnv1a32((const uint8_t *)state, offsetof(struct retained_time_sync_state, checksum));
}

static void retained_time_sync_clear(void)
{
	(void)memset(&retained_time_sync, 0, sizeof(retained_time_sync));
}

static void retained_time_sync_store(int64_t offset_us)
{
	struct retained_time_sync_state tmp = {0};
	tmp.magic = TIME_SYNC_RETAINED_MAGIC;
	tmp.version = TIME_SYNC_RETAINED_VERSION;
	tmp.offset_us = offset_us;
	tmp.checksum = retained_time_sync_checksum(&tmp);
	retained_time_sync = tmp;
}

static bool retained_time_sync_restore(int64_t *offset_us_out)
{
	if (offset_us_out == NULL) {
		return false;
	}

	if (retained_time_sync.magic != TIME_SYNC_RETAINED_MAGIC ||
	    retained_time_sync.version != TIME_SYNC_RETAINED_VERSION) {
		return false;
	}

	if (retained_time_sync.checksum != retained_time_sync_checksum(&retained_time_sync)) {
		LOG_WRN("Discarding retained time sync offset due to checksum mismatch");
		retained_time_sync_clear();
		return false;
	}

	if (!time_offset_in_valid_range(retained_time_sync.offset_us)) {
		LOG_WRN("Discarding retained time sync offset outside valid range: %lld",
			(long long)retained_time_sync.offset_us);
		retained_time_sync_clear();
		return false;
	}

	*offset_us_out = retained_time_sync.offset_us;
	return true;
}

static int time_sync_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(name, "offset") != 0) {
		/* Continue below for additional keys. */
	} else {
		if (len != sizeof(int64_t)) {
			return -EINVAL;
		}

		int64_t offset_us = 0;
		int rc = read_cb(cb_arg, &offset_us, sizeof(offset_us));
		if (rc < 0) {
			return rc;
		}

		if (rc != sizeof(offset_us)) {
			return -EINVAL;
		}

		if (!time_offset_in_valid_range(offset_us)) {
			LOG_WRN("Ignoring persisted time sync offset outside valid range: %lld",
				(long long)offset_us);
			settings_time_sync_offset_valid = false;
			return 0;
		}

		settings_time_sync_offset_us = offset_us;
		settings_time_sync_offset_valid = true;
		return 0;
	}

	if (strcmp(name, "last_unix") == 0) {
		if (len != sizeof(uint64_t)) {
			return -EINVAL;
		}

		uint64_t unix_us = 0;
		int rc = read_cb(cb_arg, &unix_us, sizeof(unix_us));
		if (rc < 0) {
			return rc;
		}

		if (rc != sizeof(unix_us)) {
			return -EINVAL;
		}

		if (!time_offset_in_valid_range((int64_t)unix_us)) {
			settings_last_unix_valid = false;
			return 0;
		}

		settings_last_unix_us = unix_us;
		settings_last_unix_valid = true;
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(time_sync_settings, "oe_time", NULL, time_sync_settings_set, NULL, NULL);

static void persist_time_sync_offset_to_settings(int64_t offset_us)
{
	int ret = settings_save_one(TIME_SYNC_SETTINGS_KEY, &offset_us, sizeof(offset_us));
	if (ret != 0) {
		LOG_WRN("Failed to persist time sync offset to settings, ret=%d", ret);
	}
}

static void persist_last_unix_to_settings(uint64_t unix_us)
{
	int ret = settings_save_one(TIME_SYNC_SETTINGS_LAST_UNIX_KEY, &unix_us, sizeof(unix_us));
	if (ret != 0) {
		LOG_WRN("Failed to persist last unix time to settings, ret=%d", ret);
	}
}

static void clear_time_sync_offset_from_settings(void)
{
	int ret = settings_delete(TIME_SYNC_SETTINGS_KEY);
	if (ret != 0 && ret != -ENOENT) {
		LOG_WRN("Failed to clear persisted time sync offset, ret=%d", ret);
	}

	ret = settings_delete(TIME_SYNC_SETTINGS_LAST_UNIX_KEY);
	if (ret != 0 && ret != -ENOENT) {
		LOG_WRN("Failed to clear persisted unix snapshot, ret=%d", ret);
	}
}

// Work item for deferred callback execution
static struct k_work time_sync_work;

static void time_sync_work_handler(struct k_work *work) {
    if (time_sync_callback != NULL) {
        time_sync_callback();
    }
}

bool notify_rtt_enabled = false;

uint64_t oe_micros() {
    return get_current_time_us();
}

static int parse_time_sync_packet_le(const void *buf, uint16_t len, struct time_sync_packet *out)
{
    if (len != sizeof(struct time_sync_packet) || out == NULL || buf == NULL) {
        return -EINVAL;
    }

    const uint8_t *p = (const uint8_t *)buf;
    out->version  = p[0];
    out->op       = p[1];
    out->seq      = sys_get_le16(&p[2]);
    out->t1_phone = sys_get_le64(&p[4]);
    out->t2_dev_rx = sys_get_le64(&p[12]);
    out->t3_dev_tx = sys_get_le64(&p[20]);
    return 0;
}

static ssize_t write_rtt_request(
    struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,
    uint8_t flags
) {
    uint64_t rx_time = get_current_time_us();

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(struct time_sync_packet)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    struct time_sync_packet pkt = {0};
    int pret = parse_time_sync_packet_le(buf, len, &pkt);
    if (pret != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    LOG_DBG("Received time sync RTT request, len: %u, handle: %u, conn: %p", len, attr->handle, (void *)conn);
    LOG_DBG("Request data: version: %u, op: %u, seq: %u, t1_phone: %llu, t2_dev_rx: %llu, t3_dev_tx: %llu",
        pkt.version,
        pkt.op,
        pkt.seq,
        pkt.t1_phone,
        pkt.t2_dev_rx,
        pkt.t3_dev_tx
    );

    if (pkt.version != 1) {
        LOG_ERR("Unsupported time sync packet version: %u", pkt.version);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    if (pkt.op != TIME_SYNC_OP_REQUEST) {
        LOG_ERR("Unsupported time sync packet operation: %u", pkt.op);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    pkt.op = TIME_SYNC_OP_RESPONSE;
    pkt.t2_dev_rx = rx_time;
    pkt.t3_dev_tx = get_current_time_us();

    if (notify_rtt_enabled) {
        (void)bt_gatt_notify(conn, attr, &pkt, sizeof(pkt));
    }

    return len;
}

static ssize_t write_time_offset(
    struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,
    uint8_t flags
) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(int64_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    int64_t delta;
    memcpy(&delta, buf, sizeof(delta));
    time_sync_apply_offset(delta);

    return len;
}

void time_sync_register_callback(time_sync_callback_t callback) {
    time_sync_callback = callback;
}

static void notify_synced(void) {
    if (!time_synced) {
        time_synced = true;
        LOG_INF("Time synchronized");
        if (time_sync_callback != NULL) {
            k_work_submit_to_queue(&sensor_work_q, &time_sync_work);
        }
    }
}

void time_sync_set_offset(int64_t offset_us) {
    time_offset_us = offset_us;
    LOG_DBG("Time offset set to: %lld us", time_offset_us);
    if (time_offset_in_valid_range(time_offset_us)) {
        retained_time_sync_store(time_offset_us);
        persist_time_sync_offset_to_settings(time_offset_us);
        persist_last_unix_to_settings(get_current_time_us());
        last_snapshot_persist_us = get_time_since_boot_us();
    } else {
        retained_time_sync_clear();
        clear_time_sync_offset_from_settings();
    }
    notify_synced();
}

void time_sync_apply_offset(int64_t delta_us) {
    time_offset_us += delta_us;
    LOG_DBG("Time offset adjusted by %lld us, new offset: %lld us", delta_us, time_offset_us);
    if (time_offset_in_valid_range(time_offset_us)) {
        retained_time_sync_store(time_offset_us);
        persist_time_sync_offset_to_settings(time_offset_us);
        persist_last_unix_to_settings(get_current_time_us());
        last_snapshot_persist_us = get_time_since_boot_us();
    } else {
        retained_time_sync_clear();
        clear_time_sync_offset_from_settings();
    }
    notify_synced();
}

bool time_sync_is_synced(void) {
    return time_synced;
}

void time_sync_persist_now_snapshot(void)
{
	if (!time_synced) {
		return;
	}

	uint64_t now_uptime_us = get_time_since_boot_us();
	if ((now_uptime_us - last_snapshot_persist_us) < TIME_SYNC_SNAPSHOT_PERSIST_INTERVAL_US) {
		return;
	}

	last_snapshot_persist_us = now_uptime_us;
	persist_last_unix_to_settings(get_current_time_us());
}

bool can_sync_time() {
    //TODO: implement check if sensors are running that prevent time sync   
    return true;
}

int init_time_sync(void) {
	k_work_init(&time_sync_work, time_sync_work_handler);
	
	// Initialize USB time sync
	usb_time_sync_init();

	if (oe_boot_state.manual_reset) {
		LOG_INF("Manual reset detected: clearing persisted time sync state");
		retained_time_sync_clear();
		clear_time_sync_offset_from_settings();
		settings_time_sync_offset_valid = false;
		settings_last_unix_valid = false;
		time_offset_us = 0;
		time_synced = false;
		return 0;
	}

	if (settings_last_unix_valid) {
		int64_t derived_offset = (int64_t)settings_last_unix_us - (int64_t)get_time_since_boot_us();
		if (time_offset_in_valid_range(derived_offset)) {
			time_offset_us = derived_offset;
			LOG_INF("Restored time sync from persisted unix snapshot: %lld us",
				(long long)settings_last_unix_us);
			retained_time_sync_store(time_offset_us);
			notify_synced();
			return 0;
		}
	}

	if (settings_time_sync_offset_valid) {
		time_offset_us = settings_time_sync_offset_us;
		LOG_INF("Restored persisted time sync offset: %lld us", (long long)settings_time_sync_offset_us);
		notify_synced();
	} else {
		int64_t retained_offset_us = 0;
		if (retained_time_sync_restore(&retained_offset_us)) {
			time_offset_us = retained_offset_us;
			LOG_INF("Restored retained time sync offset: %lld us", (long long)retained_offset_us);
			persist_time_sync_offset_to_settings(retained_offset_us);
			notify_synced();
		} else {
			LOG_INF("Boot startup: no persisted time sync offset found");
		}
	}
	
	return 0;
}

inline uint64_t get_current_time_us() {
   uint64_t base_u = get_time_since_boot_us();
   int64_t base_s = (base_u > (uint64_t)INT64_MAX) ? INT64_MAX : (int64_t)base_u;
   int64_t now_s = base_s + time_offset_us;
   if (now_s < 0) {
       LOG_WRN("Current time underflow, returning 0");
       return 0;
    }
    if (now_s != base_u + time_offset_us) {
        LOG_WRN("Current time overflow, returning UINT64_MAX");
        return UINT64_MAX;
    }
    return (uint64_t)now_s;
}

inline uint64_t get_time_since_boot_us() {
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

void rtt_cfg_changed(const struct bt_gatt_attr *attr,
                  uint16_t value) {
    LOG_DBG("RTT characteristic CCCD changed: %u", value);
    notify_rtt_enabled = (value == BT_GATT_CCC_NOTIFY);
}


BT_GATT_SERVICE_DEFINE(time_sync_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_TIME_SYNC_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_TIME_SYNC_OFFSET_CHARAC,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL, write_time_offset, &time_offset_us),
    BT_GATT_CHARACTERISTIC(BT_UUID_TIME_SYNC_RTT_CHARAC,
                BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                BT_GATT_PERM_WRITE,
                NULL, write_rtt_request, NULL),
    BT_GATT_CCC(rtt_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
