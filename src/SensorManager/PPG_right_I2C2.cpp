#include "PPG_right_I2C2.h"
#include "SensorManager.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MAXM86161);

// Static configuration for PPG right sensor
const PPGSensorConfig PPG_right_I2C2::ppg_config = {
    .i2c_dev = &I2C2,
    .i2c_addr = DT_REG_ADDR(DT_NODELABEL(maxm86161_right)),
    .sensor_id = ID_PPG_right_I2C2,
    .name = "right"
};

// Static instance
PPG_right_I2C2 PPG_right_I2C2::sensor;

PPG_right_I2C2::PPG_right_I2C2() : PPGSensor(ppg_config) {
}

bool PPG_right_I2C2::init(struct k_msgq* queue) {
    // Set up callbacks before calling base init
    set_callbacks(sensor_timer_handler, update_sensor);
    return PPGSensor::init(queue);
}

void PPG_right_I2C2::update_sensor(struct k_work* work) {
    sensor.do_update_sensor();
}

void PPG_right_I2C2::sensor_timer_handler(struct k_timer* dummy) {
    k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}
