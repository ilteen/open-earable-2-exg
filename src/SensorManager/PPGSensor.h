#ifndef _PPG_SENSOR_H
#define _PPG_SENSOR_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <array>
#include <utility>

#include "MAXM86161/MAXM86161.h"
#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

// Forward declaration
class TWIM;

/**
 * @brief Configuration struct for PPG sensor instances
 */
struct PPGSensorConfig {
    TWIM* i2c_dev;                   // I2C device (I2C2 or I2C3)
    uint8_t i2c_addr;                // I2C address from device tree
    uint8_t sensor_id;               // Sensor ID (ID_PPG_right_I2C2 or ID_PPG_left_I2C3)
    const char* name;                // Human-readable name for logging
};

/**
 * @brief Base class for MAXM86161 PPG sensors
 * 
 * This class implements all the common PPG sensor logic.
 * Subclasses only need to provide configuration via PPGSensorConfig.
 */
class PPGSensor : public EdgeMlSensor {
public:
    PPGSensor(const PPGSensorConfig& config);

    bool init(struct k_msgq* queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<16> sample_rates;

    // Static callback wrappers - each instance needs its own
    // These will be set by subclasses
    void set_callbacks(void (*timer_handler)(struct k_timer*), 
                       void (*work_handler)(struct k_work*));

    // Called from the static work handler
    void do_update_sensor();

protected:
    MAXM86161 ppg;
    const PPGSensorConfig& config;
    
    ppg_sample data_buffer[64];
    float t_sample_us;
    bool _active = false;
    int _num_samples_buffered;
    float _sample_count = 0;
    uint64_t _last_time_stamp = 0;

    struct sensor_msg msg_ppg;
};

#endif
