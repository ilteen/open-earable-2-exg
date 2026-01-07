#include "PPGSensor.h"
#include "SensorManager.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MAXM86161);

// Shared sample rates for all PPG sensors
const SampleRateSetting<16> PPGSensor::sample_rates = {
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0A, 0x0B,
    0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 },

    { 25, 50, 84, 100, 200, 400, 8, 16,
    32, 64, 128, 256, 512, 1024, 2048, 4096 },

    { 24.995, 50.027, 84.021, 99.902, 199.805, 399.610, 8.000, 16.000,
    32.000, 64.000, 128.000, 256.000, 512.000, 1024.000, 2048.000, 4096.000},
};

PPGSensor::PPGSensor(const PPGSensorConfig& cfg) 
    : config(cfg), ppg(cfg.i2c_dev, cfg.i2c_addr) {
}

void PPGSensor::set_callbacks(void (*timer_handler)(struct k_timer*), 
                              void (*work_handler)(struct k_work*)) {
    k_work_init(&sensor_work, work_handler);
    k_timer_init(&sensor_timer, timer_handler, NULL);
}

bool PPGSensor::init(struct k_msgq* queue) {
    if (!_active) {
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);

        const struct gpio_dt_spec LDO_EN = {
            .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
            .pin = 6,
            .dt_flags = GPIO_ACTIVE_HIGH
        };

        int ret = gpio_pin_configure_dt(&LDO_EN, GPIO_OUTPUT_ACTIVE);
        if (ret != 0) {
            LOG_WRN("Failed to set GPOUT as input.");
            return false;
        }

        k_msleep(5);

        _active = true;
    }
    
    if (ppg.init() != 0) {
        LOG_WRN("Could not find a valid PPG sensor (%s), check wiring!", config.name);
        _active = false;
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
        return false;
    }

    sensor_queue = queue;

    return true;
}

void PPGSensor::do_update_sensor() {
    int int_status;
    int status;

    uint64_t _time_stamp = micros();

    _sample_count += (_time_stamp - _last_time_stamp) / t_sample_us;
    _last_time_stamp = _time_stamp;

    if (_sample_count < _num_samples_buffered * (1.f - CONFIG_SENSOR_CLOCK_ACCURACY / 100.f)) {
        return;
    }
    
    status = ppg.read_interrupt_state(int_status);
    
    if (status != 0) {
        LOG_ERR("PPG read interrupt state failed (%s): %d", config.name, status);
        return;
    }
    
    if (int_status & MAXM86161_INT_FULL) {
        int num_samples = ppg.read(data_buffer);

        _sample_count = MAX(0, _num_samples_buffered - num_samples);

        int written = 0;
        const int _size = 4 * sizeof(uint32_t); // red, ir, green, ambient

        while (written < num_samples) {
            int to_write = MIN((SENSOR_DATA_FIXED_LENGTH - sizeof(uint16_t)) / _size, num_samples - written);
            if (to_write <= 0) break;

            msg_ppg.sd = _sd_logging;
            msg_ppg.stream = _ble_stream;

            msg_ppg.data.id = config.sensor_id;
            msg_ppg.data.size = to_write * _size + sizeof(uint16_t);

            const uint64_t dt_us = (uint64_t)((double)(num_samples - written) * (double)t_sample_us);
            msg_ppg.data.time = _time_stamp - dt_us;

            if (to_write > 1) {
                uint16_t t_diff = t_sample_us;
                for (int i = 0; i < to_write; i++) {
                    memcpy(&msg_ppg.data.data[i * _size], &data_buffer[written + i], _size);
                }
                memcpy(&msg_ppg.data.data[msg_ppg.data.size - sizeof(uint16_t)], &t_diff, sizeof(uint16_t));
            } else {
                memcpy(&msg_ppg.data.data, &data_buffer[written], _size);
            }

            int ret = k_msgq_put(sensor_queue, &msg_ppg, K_NO_WAIT);
            if (ret) {
                LOG_WRN("sensor msg queue full");
            }

            written += to_write;
        }
    }
}

void PPGSensor::start(int sample_rate_idx) {
    if (!_active) return;

    t_sample_us = 1e6 / sample_rates.true_sample_rates[sample_rate_idx];

    k_timeout_t t = K_USEC(t_sample_us);

    _num_samples_buffered = MIN(MAX(1, (int)(CONFIG_SENSOR_LATENCY_MS * 1e3 / t_sample_us)), FIFO_SIZE / LED_NUM - 2);
    
    ppg.set_interrogation_rate(sample_rates.reg_vals[sample_rate_idx]);
    ppg.set_watermark(FIFO_SIZE - _num_samples_buffered * LED_NUM);
    ppg.start();

    k_timer_start(&sensor_timer, K_NO_WAIT, t);

    _running = true;
    _sample_count = 0;
    _last_time_stamp = micros();
}

void PPGSensor::stop() {
    if (!_active) return;
    _active = false;

    _running = false;

    k_timer_stop(&sensor_timer);

    ppg.stop();

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
}
