#include "Temp_right_I2C2.h"
#include "SensorManager.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MLX90632);

// Static configuration for temperature right sensor
const TempSensorConfig Temp_right_I2C2::temp_config = {
    .i2c_dev = &I2C2,
    .i2c_addr = DT_REG_ADDR(DT_NODELABEL(mlx90632_right)),
    .sensor_id = ID_OPTTEMP_right_I2C2,
    .name = "right"
};

// Static instance
Temp_right_I2C2 Temp_right_I2C2::sensor;

Temp_right_I2C2::Temp_right_I2C2() : TempSensor(temp_config) {
}

bool Temp_right_I2C2::init(struct k_msgq* queue) {
    // Set up callbacks before calling base init
    set_callbacks(sensor_timer_handler, update_sensor);
    return TempSensor::init(queue);
}

void Temp_right_I2C2::update_sensor(struct k_work* work) {
    sensor.do_update_sensor();
}

void Temp_right_I2C2::sensor_timer_handler(struct k_timer* dummy) {
    k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}
