#include "ExG.h"

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <stdlib.h>

#include "SensorManager.h"
#include "SensorScheme.h"

LOG_MODULE_REGISTER(ExG, LOG_LEVEL_INF);

namespace {
constexpr gpio_pin_t PIN_AFE_CS = 25;
constexpr gpio_pin_t PIN_AFE_SCK = 6;
constexpr gpio_pin_t PIN_AFE_MOSI = 13;
constexpr gpio_pin_t PIN_AFE_MISO = 12;
constexpr uint32_t FIFO_POLL_INTERVAL_MS = 4;
constexpr AFE1594::PatientBiasMode EXG_BIAS_MODE =
    AFE1594::PatientBiasMode::DedicatedRld;
constexpr AFE1594::ChannelGain EXG_CHANNEL_GAIN = AFE1594::ChannelGain::Gain9;
constexpr float ADC_LSB_MICROVOLTS = 0.3365f;
constexpr size_t MAX_FRAMES_PER_DRAIN = 4;
constexpr uint8_t OUTPUT_DECIMATION[] = {5, 1, 1, 1, 1, 1, 1};

atomic_t sample_count;
float last_sample_microvolts[AFE1594::ECG_CHANNEL_COUNT];
int32_t minimum_raw_sample[AFE1594::ECG_CHANNEL_COUNT];
int32_t maximum_raw_sample[AFE1594::ECG_CHANNEL_COUNT];
} // namespace

ExG ExG::sensor(0, ID_EXG, "Eartips");
ExG ExG::sensor_3_4(1, ID_EXG_3_4, "3_4");
ExG ExG::sensor_5_6(2, ID_EXG_5_6, "5_6");
ExG ExG::sensor_7_8(3, ID_EXG_7_8, "7_8");
ExG *ExG::channels[AFE1594::ECG_CHANNEL_COUNT] = {
    &sensor, &sensor_3_4, &sensor_5_6, &sensor_7_8,
};

AFE1594 ExG::afe;
struct k_timer ExG::poll_timer;
struct k_work ExG::poll_work;
bool ExG::hardware_active = false;
uint8_t ExG::active_channel_count = 0;
int ExG::current_sample_rate_index = -1;
uint32_t ExG::hardware_sample_period_us = 4000U;

/* The AFE minimum is 125 SPS; 50 Hz is produced by decimating 250 SPS by 5. */
const SampleRateSetting<7> ExG::sample_rates = {
    {4, 5, 4, 3, 2, 1, 0},
    {50, 125, 250, 500, 1000, 2000, 4000},
    {50.0f, 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f},
};

ExG::ExG(uint8_t channel_index, uint8_t sensor_id, const char *name)
    : channel_index(channel_index), sensor_id(sensor_id), name(name)
{
}

bool ExG::init(struct k_msgq *queue)
{
    sensor_queue = queue;
    if (hardware_active) {
        _active = true;
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

    k_msleep(100);
    k_work_init(&poll_work, update_sensor);
    k_timer_init(&poll_timer, poll_timer_handler, nullptr);

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
        !afe.configureDifferentialChannels(EXG_CHANNEL_GAIN)) {
        LOG_ERR("AFE1594 initialization failed");
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        return false;
    }

    hardware_active = true;
    _active = true;
    LOG_INF("AFE1594 differential channels ready, dedicated RLD, gain %.2f",
            (double)AFE1594::gainValue(EXG_CHANNEL_GAIN));
    return true;
}

void ExG::poll_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    if (active_channel_count > 0U) {
        k_work_submit_to_queue(&sensor_work_q, &poll_work);
    }
}

void ExG::publish_sample(int32_t raw_sample, uint64_t timestamp)
{
    const float microvolts =
        (raw_sample * ADC_LSB_MICROVOLTS) / AFE1594::gainValue(EXG_CHANNEL_GAIN);
    if (raw_sample < minimum_raw_sample[channel_index]) {
        minimum_raw_sample[channel_index] = raw_sample;
    }
    if (raw_sample > maximum_raw_sample[channel_index]) {
        maximum_raw_sample[channel_index] = raw_sample;
    }

    decimation_sum += microvolts;
    if (++decimation_phase < output_decimation) {
        return;
    }

    const float output_microvolts = decimation_sum / output_decimation;
    decimation_phase = 0;
    decimation_sum = 0.0f;
    last_sample_microvolts[channel_index] = output_microvolts;
    atomic_inc(&sample_count);

    struct sensor_msg message = {};
    message.sd = _sd_logging;
    message.stream = _ble_stream;
    message.data.id = sensor_id;
    message.data.size = sizeof(output_microvolts);
    message.data.time = timestamp;
    memcpy(message.data.data, &output_microvolts, sizeof(output_microvolts));

    if (k_msgq_put(sensor_queue, &message, K_NO_WAIT) != 0) {
        LOG_WRN("sensor msg queue full (%s)", name);
    }
}

void ExG::update_sensor(struct k_work *work)
{
    ARG_UNUSED(work);

    int32_t frames[MAX_FRAMES_PER_DRAIN][AFE1594::ECG_CHANNEL_COUNT];
    bool overflow = false;
    size_t frame_count;

    do {
        frame_count = afe.readSampleFrames(frames, ARRAY_SIZE(frames), &overflow);
        if (overflow) {
            LOG_WRN("AFE1594 Main FIFO overflow");
        }
        if (frame_count == 0U) {
            break;
        }

        const uint64_t now = micros();
        for (size_t frame = 0; frame < frame_count; ++frame) {
            const uint64_t timestamp =
                now - ((frame_count - 1U - frame) * hardware_sample_period_us);
            for (size_t channel = 0; channel < AFE1594::ECG_CHANNEL_COUNT; ++channel) {
                if (channels[channel]->_running) {
                    channels[channel]->publish_sample(frames[frame][channel], timestamp);
                }
            }
        }
    } while (active_channel_count > 0U && frame_count == ARRAY_SIZE(frames));
}

void ExG::start(int sample_rate_idx)
{
    if (!_active || _running || sample_rate_idx < 0 ||
        sample_rate_idx >= static_cast<int>(ARRAY_SIZE(sample_rates.reg_vals))) {
        return;
    }
    if (active_channel_count > 0U && sample_rate_idx != current_sample_rate_index) {
        LOG_ERR("ExG %s requires the active shared sample-rate index %d",
                name, current_sample_rate_index);
        return;
    }

    output_decimation = OUTPUT_DECIMATION[sample_rate_idx];
    decimation_phase = 0;
    decimation_sum = 0.0f;
    hardware_sample_period_us = static_cast<uint32_t>(
        1000000.0f /
        (sample_rates.true_sample_rates[sample_rate_idx] * output_decimation));
    minimum_raw_sample[channel_index] = INT32_MAX;
    maximum_raw_sample[channel_index] = INT32_MIN;

    if (active_channel_count == 0U) {
        atomic_set(&sample_count, 0);
        if (!afe.setDataRate(sample_rates.reg_vals[sample_rate_idx]) ||
            !afe.startStreaming()) {
            LOG_ERR("Failed to start AFE1594 streaming");
            return;
        }
        current_sample_rate_index = sample_rate_idx;
        k_timer_start(&poll_timer, K_NO_WAIT, K_MSEC(FIFO_POLL_INTERVAL_MS));
    }

    _running = true;
    active_channel_count++;
    LOG_INF("ExG %s started at %.1f SPS", name,
            (double)sample_rates.true_sample_rates[sample_rate_idx]);
}

void ExG::stop()
{
    _active = false;
    if (!_running) {
        return;
    }

    _running = false;
    if (active_channel_count > 0U) {
        active_channel_count--;
    }
    if (active_channel_count > 0U) {
        return;
    }

    k_timer_stop(&poll_timer);
    struct k_work_sync sync;
    k_work_cancel_sync(&poll_work, &sync);
    afe.stopStreaming();
    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
    hardware_active = false;
    current_sample_rate_index = -1;
    LOG_INF("AFE1594 stopped");
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
    shell_print(shell, "AFE1594 Eartips start queued at %.0f Hz",
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
    shell_print(shell, "AFE1594 Eartips stop queued");
    return 0;
}

static int cmd_exg_status(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell,
                "running=[%d,%d,%d,%d] samples=%ld last=[%.3f,%.3f,%.3f,%.3f] uV id=0x%06x spi=%d",
                ExG::sensor.is_running(), ExG::sensor_3_4.is_running(),
                ExG::sensor_5_6.is_running(), ExG::sensor_7_8.is_running(),
                (long)atomic_get(&sample_count),
                (double)last_sample_microvolts[0],
                (double)last_sample_microvolts[1],
                (double)last_sample_microvolts[2],
                (double)last_sample_microvolts[3],
                (unsigned int)ExG::last_design_id(), ExG::last_spi_error());
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(exg_cmd,
    SHELL_CMD(start, NULL, "Start AFE1594 Eartips stream [rate index 0-6]", cmd_exg_start),
    SHELL_CMD(stop, NULL, "Stop AFE1594 Eartips stream", cmd_exg_stop),
    SHELL_CMD(status, NULL, "Show AFE1594 stream status", cmd_exg_status),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(exg, &exg_cmd, "AFE1594 ExG diagnostics", NULL);
