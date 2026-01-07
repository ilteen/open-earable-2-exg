#include "TempSensor.h"
#include "SensorManager.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MLX90632);

// Shared sample rates for all temperature sensors
const SampleRateSetting<8> TempSensor::sample_rates = {
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 },  // reg_vals
    { 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0 },       // sample_rates
    { 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0 }        // true_sample_rates
};

TempSensor::TempSensor(const TempSensorConfig& cfg) 
    : config(cfg) {
}

void TempSensor::set_callbacks(void (*timer_handler)(struct k_timer*), 
                               void (*work_handler)(struct k_work*)) {
    k_work_init(&sensor_work, work_handler);
    k_timer_init(&sensor_timer, timer_handler, NULL);
}

bool TempSensor::init(struct k_msgq* queue) {
    if (!_active) {
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);
        _active = true;
    }

    MLX90632::status returnError;
    if (!temp.begin(config.i2c_addr, *config.i2c_dev, returnError)) {
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);

        _active = false;

        LOG_WRN("Could not find a valid Optical Temperature sensor (%s), check wiring!", config.name);
        return false;
    }

    sensor_queue = queue;

    _active = true;

    return true;
}

void TempSensor::do_update_sensor() {
    if (!temp.dataAvailable()) return;

    MLX90632::status returnError;
    float temperature = temp.getObjectTemp(returnError);

    if (returnError != MLX90632::SENSOR_SUCCESS) {
        LOG_WRN("Error reading temperature (%s)", config.name);
        return;
    }

    msg_temp.sd = _sd_logging;
    msg_temp.stream = _ble_stream;

    msg_temp.data.id = config.sensor_id;
    msg_temp.data.size = sizeof(float);
    msg_temp.data.time = micros();

    memcpy(msg_temp.data.data, &temperature, sizeof(float));

    int ret = k_msgq_put(sensor_queue, &msg_temp, K_NO_WAIT);
    if (ret) {
        LOG_WRN("sensor msg queue full");
    }
}

void TempSensor::start(int sample_rate_idx) {
    if (!_active) return;

    k_timeout_t t = K_USEC(1e6 / sample_rates.true_sample_rates[sample_rate_idx]);

    temp.setSampleRateRegVal(sample_rates.reg_vals[sample_rate_idx]);
    temp.continuousMode();

    k_timer_start(&sensor_timer, K_NO_WAIT, t);

    _running = true;
}

void TempSensor::stop() {
    if (!_active) return;
    _active = false;

    _running = false;

    k_timer_stop(&sensor_timer);

    temp.sleepMode();

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
}
