#ifndef _EXG_H
#define _EXG_H

#include <zephyr/kernel.h>
#include <array>
#include <utility>

#include "AFE1594/AFE1594.h"
#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

class ExG : public EdgeMlSensor {
public:
    static ExG sensor;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    static uint32_t last_design_id() { return afe.lastDesignIdRead(); }
    static int last_spi_error() { return afe.lastSpiError(); }

    const static SampleRateSetting<7> sample_rates;

private:
    static AFE1594 afe;
    static struct k_timer poll_timer;

    static void poll_timer_handler(struct k_timer *timer);
    static void update_sensor(struct k_work *work);

    bool _active = false;
    uint32_t sample_period_us = 4000U;
    uint8_t output_decimation = 1U;
    uint8_t decimation_phase = 0U;
    float decimation_sum = 0.0f;
};

#endif // _EXG_H
