#ifndef _EXG_H
#define _EXG_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <array>

#include "AD7124/AD7124.h"
#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

class ExG : public EdgeMlSensor {
public:
    static ExG sensor;

    bool init(struct k_msgq *queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    // Sample rate table: 8 entries
    // reg_vals        : FS register words written to AD7124 filter register
    // output_data_rate: ADC output rate (both channels combined, SPS)
    // true_sample_rates: effective per-channel SPS = output_data_rate / 2
    //                    (two channels time-multiplexed; ADC alternates CH0/CH1)
    const static SampleRateSetting<8> sample_rates;

    static constexpr uint8_t DEFAULT_SAMPLE_RATE_IDX = 1;

private:
    static AD7124 *adc;

    // static void sensor_timer_handler(struct k_timer *dummy);
    static void update_sensor(struct k_work *work);

    static void rdy_isr(const struct device *port, struct gpio_callback *cb ,uint32_t pins);

    static struct gpio_callback rdy_cb;

    static float ch_val[2];
    static bool ch_ready[2];

    static constexpr uint8_t BATCH_SIZE = 4;

    struct __attribute__((packed)) SamplePair {
        float ch0; // micro volts
        float ch1; // micro volts
        uint8_t dt_ms; // delta since batch_start_time, in ms
    };

    static SamplePair batch[BATCH_SIZE];
    static uint8_t batch_count;
    static uint64_t batch_start_time; // in ms

    bool _active = false;
};

#endif // _EXG_H
