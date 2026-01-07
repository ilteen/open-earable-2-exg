#ifndef _PPG_LEFT_I2C3_H
#define _PPG_LEFT_I2C3_H

#include "PPGSensor.h"

/**
 * @brief PPG sensor on I2C3 (left ear)
 * 
 * This is a thin wrapper around PPGSensor that provides
 * the static instance and configuration for the left PPG sensor.
 */
class PPG_left_I2C3 : public PPGSensor {
public:
    PPG_left_I2C3();

    static PPG_left_I2C3 sensor;

    bool init(struct k_msgq* queue) override;

    // Re-export sample_rates from base class for compatibility
    using PPGSensor::sample_rates;

private:
    static const PPGSensorConfig ppg_config;
    
    static void sensor_timer_handler(struct k_timer* dummy);
    static void update_sensor(struct k_work* work);
};

#endif
