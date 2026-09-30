#ifndef BARO_H
#define BARO_H

#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

#include "BMP388/Adafruit_BMP3XX.h"

struct BaroConfig {
    TWIM *i2c_dev;
    uint8_t i2c_addr;
    uint8_t sensor_id;
    const char *name;
};

class Baro : public EdgeMlSensor {
public:
    explicit Baro(const BaroConfig& config);

    static Baro right;
    static Baro left;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<18> sample_rates;
private:
    Adafruit_BMP3XX bmp;
    const BaroConfig& config;
    struct sensor_msg msg_baro;
    int initial_discard = 1;

    bool _active = false;

    void do_update_sensor();

    static void update_right(struct k_work *work);
    static void update_left(struct k_work *work);
    static void right_timer_handler(struct k_timer *dummy);
    static void left_timer_handler(struct k_timer *dummy);
};

#endif
