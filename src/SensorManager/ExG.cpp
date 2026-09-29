#include "ExG.h"

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <stdlib.h>

#include "SensorManager.h"
#include "SensorScheme.h"

LOG_MODULE_REGISTER(ExG, LOG_LEVEL_INF);

namespace {
constexpr gpio_pin_t PIN_AFE_CS = 25;  // Extension IO1 / P0.25
constexpr gpio_pin_t PIN_AFE_SCK = 6;  // Extension IO4 / P0.06
constexpr gpio_pin_t PIN_AFE_MOSI = 13; // Extension IO5 / P0.13
constexpr gpio_pin_t PIN_AFE_MISO = 12; // Extension IO6 / P0.12
constexpr uint32_t FIFO_POLL_INTERVAL_MS = 4;

/* Change only this line to select the complete patient-bias configuration. */
constexpr AFE1594::PatientBiasMode EXG_BIAS_MODE =
    AFE1594::PatientBiasMode::SharedE2Rld;

const char *biasModeName()
{
    switch (EXG_BIAS_MODE) {
    case AFE1594::PatientBiasMode::PassiveTwoElectrode: return "passive";
    case AFE1594::PatientBiasMode::DedicatedRld: return "dedicated RLD";
    case AFE1594::PatientBiasMode::SharedE2Rld: return "shared E2 RLD";
    }
    return "unknown";
}

/*
 * Change this value to select the AFE1594 channel gain. Available values are
 * Gain1p25, Gain2, Gain3, Gain4, Gain5, Gain6p5, and Gain9. Gain9 gives the
 * AFE's lowest input-referred noise for EEG while its +/-220 mV differential
 * range and common-mode range still cover the passive two-electrode bias.
 */
constexpr AFE1594::ChannelGain EXG_CHANNEL_GAIN = AFE1594::ChannelGain::Gain9;

/* AFE ADC LSB at AVDD=3.3 V, from datasheet section 9.3.6. */
constexpr float ADC_LSB_MICROVOLTS = 0.3365f;
constexpr size_t MAX_SAMPLES_PER_DRAIN = 16;

atomic_t sample_count;
float last_sample_microvolts;
int32_t minimum_raw_sample;
int32_t maximum_raw_sample;
} // namespace

ExG ExG::sensor;
AFE1594 ExG::afe;
struct k_timer ExG::poll_timer;

static struct sensor_msg msg_exg;

/* The AFE minimum is 125 SPS; 50 Hz is produced by decimating 250 SPS by 5. */
const SampleRateSetting<7> ExG::sample_rates = {
    {4, 5, 4, 3, 2, 1, 0},
    {50, 125, 250, 500, 1000, 2000, 4000},
    {50.0f, 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f},
};

constexpr uint8_t OUTPUT_DECIMATION[] = {5, 1, 1, 1, 1, 1, 1};

bool ExG::init(struct k_msgq *queue)
{
    if (_active) {
        sensor_queue = queue;
        return true;
    }

    int ret = pm_device_runtime_get(ls_1_8);
    if (ret != 0) {
        LOG_ERR("Failed to enable the 1.8 V sensor rail: %d", ret);
        return false;
    }
    ret = pm_device_runtime_get(ls_3_3);
    if (ret != 0) {
        LOG_ERR("Failed to enable the 3.3 V sensor rail: %d", ret);
        pm_device_runtime_put(ls_1_8);
        return false;
    }

    /* AFE1594 datasheet table 10-2 requires 100 ms after its rails ramp. */
    k_msleep(100);

    k_work_init(&sensor.sensor_work, update_sensor);
    k_timer_init(&poll_timer, poll_timer_handler, nullptr);
    sensor_queue = queue;

    const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    const AFE1594::BusPins pins = {
        .gpio = gpio0,
        .cs = PIN_AFE_CS,
        .sck = PIN_AFE_SCK,
        .mosi = PIN_AFE_MOSI,
        .miso = PIN_AFE_MISO,
    };

    if (!afe.begin(0, pins) ||
        !afe.configurePatientBias(EXG_BIAS_MODE) ||
        !afe.configureAveragedChannels(EXG_CHANNEL_GAIN)) {
        LOG_ERR("AFE1594 initialization failed");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        return false;
    }

    _active = true;
    LOG_INF("AFE1594 averaged E1-E3/E2-E4, gain %.2f, bias=%s, FIFO poll=%u ms",
            (double)AFE1594::gainValue(EXG_CHANNEL_GAIN),
            biasModeName(),
            FIFO_POLL_INTERVAL_MS);
    return true;
}

void ExG::poll_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    if (sensor._running) {
        k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
    }
}

void ExG::update_sensor(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!sensor._running) {
        return;
    }

    int32_t samples[MAX_SAMPLES_PER_DRAIN];
    bool overflow = false;
    size_t count;

    do {
        count = afe.readSamples(samples, ARRAY_SIZE(samples), &overflow);
        if (overflow) {
            LOG_WRN("AFE1594 Main FIFO overflow");
        }
        if (count == 0U) {
            break;
        }

        const uint64_t now = micros();
        for (size_t i = 0; i < count; ++i) {
            const float microvolts =
                (samples[i] * ADC_LSB_MICROVOLTS) /
                AFE1594::gainValue(EXG_CHANNEL_GAIN);
            if (samples[i] < minimum_raw_sample) {
                minimum_raw_sample = samples[i];
            }
            if (samples[i] > maximum_raw_sample) {
                maximum_raw_sample = samples[i];
            }
            sensor.decimation_sum += microvolts;
            if (++sensor.decimation_phase < sensor.output_decimation) {
                continue;
            }
            const float output_microvolts =
                sensor.decimation_sum / sensor.output_decimation;
            sensor.decimation_phase = 0;
            sensor.decimation_sum = 0.0f;
            last_sample_microvolts = output_microvolts;
            const atomic_val_t count_after_sample = atomic_inc(&sample_count) + 1;
            if (count_after_sample == 1) {
                LOG_INF("First AFE1594 output: %.3f uV",
                        (double)output_microvolts);
            }

            msg_exg.sd = sensor._sd_logging;
            msg_exg.stream = sensor._ble_stream;
            msg_exg.data.id = ID_EXG;
            msg_exg.data.size = sizeof(float);
            msg_exg.data.time = now - ((count - 1U - i) * sensor.sample_period_us);
            memcpy(msg_exg.data.data, &output_microvolts, sizeof(output_microvolts));

            if (k_msgq_put(sensor_queue, &msg_exg, K_NO_WAIT) != 0) {
                LOG_WRN("sensor msg queue full");
                return;
            }
        }
    } while (sensor._running && count == ARRAY_SIZE(samples));
}

void ExG::start(int sample_rate_idx)
{
    if (!_active || sample_rate_idx < 0 ||
        sample_rate_idx >= static_cast<int>(ARRAY_SIZE(sample_rates.reg_vals))) {
        return;
    }

    const uint8_t rate_selector = sample_rates.reg_vals[sample_rate_idx];
    atomic_set(&sample_count, 0);
    minimum_raw_sample = INT32_MAX;
    maximum_raw_sample = INT32_MIN;
    output_decimation = OUTPUT_DECIMATION[sample_rate_idx];
    decimation_phase = 0;
    decimation_sum = 0.0f;
    sample_period_us = static_cast<uint32_t>(
        1000000.0f /
        (sample_rates.true_sample_rates[sample_rate_idx] * output_decimation));

    if (!afe.setDataRate(rate_selector) || !afe.startStreaming()) {
        LOG_ERR("Failed to start AFE1594 streaming");
        return;
    }

    _running = true;
    k_timer_start(&poll_timer, K_NO_WAIT, K_MSEC(FIFO_POLL_INTERVAL_MS));

    LOG_INF("ExG sensor started at %.1f SPS (AFE %.1f SPS, decimation %u)",
            (double)sample_rates.true_sample_rates[sample_rate_idx],
            (double)(sample_rates.true_sample_rates[sample_rate_idx] * output_decimation),
            output_decimation);
}

void ExG::stop()
{
    if (!_active) {
        return;
    }

    _running = false;
    k_timer_stop(&poll_timer);
    struct k_work_sync sync;
    k_work_cancel_sync(&sensor_work, &sync);
    afe.stopStreaming();
    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
    _active = false;
    LOG_INF("ExG sensor stopped");
}

static int cmd_exg_start(const struct shell *shell, size_t argc, char **argv)
{
    uint8_t rate_index = 0;
    if (argc == 2) {
        char *end = nullptr;
        const unsigned long parsed = strtoul(argv[1], &end, 10);
        if (*argv[1] == '\0' || *end != '\0' ||
            parsed >= ARRAY_SIZE(ExG::sample_rates.reg_vals)) {
            shell_error(shell, "Rate index must be 0-%zu",
                        ARRAY_SIZE(ExG::sample_rates.reg_vals) - 1U);
            return -EINVAL;
        }
        rate_index = static_cast<uint8_t>(parsed);
    }

    struct sensor_config config = {
        .sensorId = ID_EXG,
        .sampleRateIndex = rate_index,
        .storageOptions = DATA_STREAMING,
    };
    config_sensor(&config);
    shell_print(shell, "AFE1594 start queued at %.0f Hz",
                (double)ExG::sample_rates.true_sample_rates[rate_index]);
    return 0;
}

static int cmd_exg_stop(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct sensor_config config = {
        .sensorId = ID_EXG,
        .sampleRateIndex = 0,
        .storageOptions = 0,
    };
    config_sensor(&config);
    shell_print(shell, "AFE1594 stop queued");
    return 0;
}

static int cmd_exg_status(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell,
                "running=%d samples=%ld raw=[%ld,%ld] last=%.3f uV id=0x%06x spi=%d",
                ExG::sensor.is_running(), (long)atomic_get(&sample_count),
                (long)minimum_raw_sample, (long)maximum_raw_sample,
                (double)last_sample_microvolts,
                (unsigned int)ExG::last_design_id(), ExG::last_spi_error());
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(exg_cmd,
    SHELL_CMD(start, NULL, "Start AFE1594 stream [rate index 0-6]", cmd_exg_start),
    SHELL_CMD(stop, NULL, "Stop AFE1594 stream", cmd_exg_stop),
    SHELL_CMD(status, NULL, "Show AFE1594 stream status", cmd_exg_status),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(exg, &exg_cmd, "AFE1594 ExG diagnostics", NULL);
