#ifndef _TEMP_LEFT_I2C3_H
#define _TEMP_LEFT_I2C3_H

#include "TempSensor.h"

/**
 * @brief Temperature sensor on I2C3 (left ear)
 * 
 * This is a thin wrapper around TempSensor that provides
 * the static instance and configuration for the left temperature sensor.
 */
class Temp_left_I2C3 : public TempSensor {
public:
    Temp_left_I2C3();

    static Temp_left_I2C3 sensor;

    bool init(struct k_msgq* queue) override;

    // Re-export sample_rates from base class for compatibility
    using TempSensor::sample_rates;

private:
    static const TempSensorConfig temp_config;
    
    static void sensor_timer_handler(struct k_timer* dummy);
    static void update_sensor(struct k_work* work);
};

#endif
