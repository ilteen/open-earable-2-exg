#ifndef _TEMP_RIGHT_I2C2_H
#define _TEMP_RIGHT_I2C2_H

#include "TempSensor.h"

/**
 * @brief Temperature sensor on I2C2 (right ear)
 * 
 * This is a thin wrapper around TempSensor that provides
 * the static instance and configuration for the right temperature sensor.
 */
class Temp_right_I2C2 : public TempSensor {
public:
    Temp_right_I2C2();

    static Temp_right_I2C2 sensor;

    bool init(struct k_msgq* queue) override;

    // Re-export sample_rates from base class for compatibility
    using TempSensor::sample_rates;

private:
    static const TempSensorConfig temp_config;
    
    static void sensor_timer_handler(struct k_timer* dummy);
    static void update_sensor(struct k_work* work);
};

#endif
