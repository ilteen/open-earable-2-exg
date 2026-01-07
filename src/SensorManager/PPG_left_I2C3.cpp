#include "PPG_left_I2C3.h"
#include "SensorManager.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MAXM86161);

// Static configuration for PPG left sensor
const PPGSensorConfig PPG_left_I2C3::ppg_config = {
    .i2c_dev = &I2C3,
    .i2c_addr = DT_REG_ADDR(DT_NODELABEL(maxm86161_left)),
    .sensor_id = ID_PPG_left_I2C3,
    .name = "left"
};

// Static instance
PPG_left_I2C3 PPG_left_I2C3::sensor;

PPG_left_I2C3::PPG_left_I2C3() : PPGSensor(ppg_config) {
}

bool PPG_left_I2C3::init(struct k_msgq* queue) {
    // Set up callbacks before calling base init
    set_callbacks(sensor_timer_handler, update_sensor);
    return PPGSensor::init(queue);
}

void PPG_left_I2C3::update_sensor(struct k_work* work) {
    sensor.do_update_sensor();
}

void PPG_left_I2C3::sensor_timer_handler(struct k_timer* dummy) {
    k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}
