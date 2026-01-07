#ifndef _PPG_RIGHT_I2C2_H
#define _PPG_RIGHT_I2C2_H

#include "PPGSensor.h"

/**
 * @brief PPG sensor on I2C2 (right ear)
 * 
 * This is a thin wrapper around PPGSensor that provides
 * the static instance and configuration for the right PPG sensor.
 */
class PPG_right_I2C2 : public PPGSensor {
public:
    PPG_right_I2C2();

    static PPG_right_I2C2 sensor;

    bool init(struct k_msgq* queue) override;

    // Re-export sample_rates from base class for compatibility
    using PPGSensor::sample_rates;

private:
    static const PPGSensorConfig ppg_config;
    
    static void sensor_timer_handler(struct k_timer* dummy);
    static void update_sensor(struct k_work* work);
};

#endif
