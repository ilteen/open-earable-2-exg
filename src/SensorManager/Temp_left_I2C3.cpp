#include "Temp_left_I2C3.h"
#include "SensorManager.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MLX90632);

// Static configuration for temperature left sensor
const TempSensorConfig Temp_left_I2C3::temp_config = {
    .i2c_dev = &I2C3,
    .i2c_addr = DT_REG_ADDR(DT_NODELABEL(mlx90632_left)),
    .sensor_id = ID_OPTTEMP_left_I2C3,
    .name = "left"
};

// Static instance
Temp_left_I2C3 Temp_left_I2C3::sensor;

Temp_left_I2C3::Temp_left_I2C3() : TempSensor(temp_config) {
}

bool Temp_left_I2C3::init(struct k_msgq* queue) {
    // Set up callbacks before calling base init
    set_callbacks(sensor_timer_handler, update_sensor);
    return TempSensor::init(queue);
}

void Temp_left_I2C3::update_sensor(struct k_work* work) {
    sensor.do_update_sensor();
}

void Temp_left_I2C3::sensor_timer_handler(struct k_timer* dummy) {
    k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}
