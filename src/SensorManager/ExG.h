#ifndef _EXG_H
#define _EXG_H

#include <zephyr/kernel.h>

#include "AFE1594/AFE1594.h"
#include "EdgeMLSensor.h"
#include "openearable_common.h"
#include "zbus_common.h"

class ExG : public EdgeMlSensor {
public:
    ExG(uint8_t channel_index, uint8_t sensor_id, const char *name);

    static ExG sensor;
    static ExG sensor_3_4;
    static ExG sensor_5_6;
    static ExG sensor_7_8;

    bool init(struct k_msgq *queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    static uint32_t last_design_id() { return afe.lastDesignIdRead(); }
    static int last_spi_error() { return afe.lastSpiError(); }

    const static SampleRateSetting<7> sample_rates;

private:
    static AFE1594 afe;
    static struct k_timer poll_timer;
    static struct k_work poll_work;
    static ExG *channels[AFE1594::ECG_CHANNEL_COUNT];
    static bool hardware_active;
    static uint8_t active_channel_count;
    static int current_sample_rate_index;
    static uint32_t hardware_sample_period_us;

    static void poll_timer_handler(struct k_timer *timer);
    static void update_sensor(struct k_work *work);
    void publish_sample(int32_t raw_sample, uint64_t timestamp);

    const uint8_t channel_index;
    const uint8_t sensor_id;
    const char *name;
    bool _active = false;
    uint8_t output_decimation = 1U;
    uint8_t decimation_phase = 0U;
    float decimation_sum = 0.0f;
};

#endif // _EXG_H
