#ifndef _TEMP_SENSOR_H
#define _TEMP_SENSOR_H

#include <zephyr/kernel.h>

#include "MLX90632/MLX90632.h"
#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

// Forward declaration
class TWIM;

/**
 * @brief Configuration struct for Temperature sensor instances
 */
struct TempSensorConfig {
    TWIM* i2c_dev;                   // I2C device (I2C2 or I2C3)
    uint8_t i2c_addr;                // I2C address from device tree
    uint8_t sensor_id;               // Sensor ID (ID_OPTTEMP_right_I2C2 or ID_OPTTEMP_left_I2C3)
    const char* name;                // Human-readable name for logging
};

/**
 * @brief Base class for MLX90632 Temperature sensors
 * 
 * This class implements all the common temperature sensor logic.
 * Subclasses only need to provide configuration via TempSensorConfig.
 */
class TempSensor : public EdgeMlSensor {
public:
    TempSensor(const TempSensorConfig& config);

    bool init(struct k_msgq* queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<8> sample_rates;

    // Static callback wrappers - each instance needs its own
    void set_callbacks(void (*timer_handler)(struct k_timer*), 
                       void (*work_handler)(struct k_work*));

    // Called from the static work handler
    void do_update_sensor();

protected:
    MLX90632 temp;
    const TempSensorConfig& config;
    
    bool _active = false;
    struct sensor_msg msg_temp;
};

#endif
