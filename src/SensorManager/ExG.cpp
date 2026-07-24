#include "ExG.h"

#include <zephyr/logging/log.h>

#include "SensorManager.h"
LOG_MODULE_REGISTER(ExG, 3);

ExG ExG::sensor;
AD7124* ExG::adc = nullptr;
struct gpio_callback ExG::rdy_cb;
float ExG::ch_val[2] = {0.0f, 0.0f};
bool ExG::ch_ready[2] = {false, false};
ExG::SamplePair ExG::batch[ExG::BATCH_SIZE];
uint8_t ExG::batch_count = 0;
uint64_t ExG::batch_start_time = 0;

static struct sensor_msg msg_exg;

// samples per second: 614400/(32*x) where x is the samplesPerSecondVal
const SampleRateSetting<8> ExG::sample_rates = {
     // FS register values (written to filter reg)
    { 160,  80,   40,   20,    10,    3,  2,   1     },
    // ADC data rate, single-channel equivalent (= 19200 / FS)
    { 120,  240,  480,  960,   1920,  6400, 9600, 19200 },
    // Measured per-channel pair SPS (= ODR / 8)
    // The sinc4 filter requires 4 reads every channel switch
    // Zero latency mode is automatically enabled when reading multiple channels
    { 15.0, 30.0, 60.0, 120.0, 240.0, 800.0, 1200, 2400.0 }
};


bool ExG::init(struct k_msgq* queue) {
    if (!_active) {
        // Power up the sensor power rails
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);

        k_msleep(5);

        _active = true;
    }

    // Get GPIO0 device for software SPI bit-banging
    const struct device* gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0)) {
        LOG_ERR("GPIO0 device not ready");
        goto fail_power;
    }

    if (adc == nullptr) {
        adc = new AD7124();
    }

    // Configure for SOFTWARE SPI (bit-banging) in 3-WIRE mode
    // Pins: SCK=P0.6, MOSI=P0.13, MISO=P0.12, CS=hardwired to GND
    adc->setSoftwareSPI(gpio0, 6, 13, 12);

    // Initialize ADC
    if (adc->init() != 0) {
        LOG_ERR("ADC init failed");
        goto fail_power;
    }

    if (adc->reset() != 0) {
        LOG_ERR("ADC reset failed");
        goto fail_power;
    }

    k_msleep(10);

    if (adc->setAdcControl(AD7124::OperatingMode::CONTINUOUS, AD7124::PowerMode::FULL_POWER, true) != 0) {
        LOG_ERR("Failed to set ADC to CONTINUOUS");
        goto fail_power;
    }

    if (adc->setConfig(0, AD7124::ReferenceSource::INTERNAL, AD7124::PGA::GAIN_1, true) != 0) {
        LOG_ERR("Failed to configure setup 0");
        goto fail_power;
    }

    /*
    if (adc->setConfig(1, AD7124::ReferenceSource::INTERNAL, AD7124::PGA::GAIN_1, true) != 0) {
        LOG_ERR("Failed to configure setup 1");
        goto fail_power;
    }
    if (adc->setFilter(1, AD7124::FilterType::SINC4, sample_rates.reg_vals[DEFAULT_SAMPLE_RATE_IDX], false, true) != 0) {
        LOG_ERR("Failed to configure filter setup 1");
        goto fail_power;
    }
    */

    k_msleep(10);

    // Configure filter for setup 0 (SINC4, 256 SPS by default)
    if (adc->setFilter(0, AD7124::FilterType::SINC4, sample_rates.reg_vals[DEFAULT_SAMPLE_RATE_IDX], false, true) != 0) {
        LOG_ERR("Failed to configure filter setup 0");
        goto fail_power;
    }
    if (adc->setChannel(0, 0, AD7124::AnalogInput::AIN0, AD7124::AnalogInput::AIN1, true) != 0) {
        LOG_ERR("Failed to configure channel 0");
        goto fail_power;
    }
    if (adc->setChannel(1, 0,
                        AD7124::AnalogInput::AIN0,
                        AD7124::AnalogInput::AIN2, true) != 0) {
        LOG_ERR("Failed to configure channel 1");
        goto fail_power;
    }

    sensor_queue = queue;

    k_work_init(&sensor.sensor_work, update_sensor);

    if (adc->enableReadyInterrupt(rdy_isr, &rdy_cb) != 0) {
        LOG_ERR("Failed to enable RDY interrupt");
        goto fail_power;
    }

    ch_ready[0] = false;
    ch_ready[1] = false;
    batch_count = 0;

    LOG_INF("ExG init OK");

    return true;

    fail_power:
    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
    _active = false;
    return false;
}


void ExG::rdy_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    adc->maskReadyInterrupt();
    k_work_submit(&sensor.sensor_work);
}

void ExG::update_sensor(struct k_work* work) {
    // Read voltage and convert to microvolts (µV)
    // InAmp gain = 50 (as per hardware design)
    static const float INAMP_GAIN = 50.0f;
    
    int32_t raw;
    uint8_t channel = 0xFF;

    int ret = adc->readRaw(&raw, &channel);
    adc->unmaskReadyInterrupt();

    if (ret != 0) {
      LOG_WRN("ADC readRaw failed: %d", ret);
      return;
    }

    if (channel > 1) {
      LOG_WRN("ADC returned invalid channel: %d", channel);
      return;
    }

    ch_val[channel] = (adc->rawToVolts(raw) / INAMP_GAIN) * 1e6f; // Convert to microvolts
    ch_ready[channel] = true;

    if (!(ch_ready[0] && ch_ready[1])) {
        // Wait for both channels to be ready
        return;
    }

    ch_ready[0] = false;
    ch_ready[1] = false;

    uint64_t now = micros();

    if (batch_count == 0) {
        batch_start_time = now;
    }

    uint64_t dt_us = now - batch_start_time;
    if (dt_us / 1000 > 255 && batch_count > 0) {
      msg_exg.stream      = sensor._ble_stream;
      msg_exg.data.id     = ID_EXG;
      msg_exg.data.size   = sizeof(SamplePair) * batch_count;
      msg_exg.data.time   = batch_start_time;
      memcpy(msg_exg.data.data, batch, msg_exg.data.size);

      if (k_msgq_put(sensor_queue, &msg_exg, K_NO_WAIT) != 0) {
          LOG_WRN("Queue full — early flush dropped");
      }

      batch_count      = 0;
      batch_start_time = now;
      dt_us            = 0;
    }

    batch[batch_count].ch0 = ch_val[0];
    batch[batch_count].ch1 = ch_val[1];
    batch[batch_count].dt_ms = static_cast<uint8_t>(dt_us / 1000);
    batch_count++;

    if (batch_count < BATCH_SIZE) {
      return;
    }

    msg_exg.stream    = sensor._ble_stream;
    msg_exg.data.id   = ID_EXG;
    msg_exg.data.size = sizeof(SamplePair) * BATCH_SIZE;
    msg_exg.data.time = batch_start_time;
    memcpy(msg_exg.data.data, batch, msg_exg.data.size);

    if (k_msgq_put(sensor_queue, &msg_exg, K_NO_WAIT) != 0) {
        LOG_WRN("Sensor queue full");
    }

    batch_count = 0;
}

void ExG::start(int sample_rate_idx) {
    if (!_active) return;

    // Update filter configuration for new sample rate
    uint16_t fs_val = sample_rates.reg_vals[sample_rate_idx];
    if ((adc->setFilter(0, AD7124::FilterType::SINC4, fs_val, false, true)) != 0){
      LOG_ERR("failed to set ADC filter setup 0 in start()");
      return;
    }

    /*if ((adc->setFilter(1, AD7124::FilterType::SINC4, fs_val, false, true)) != 0){
      LOG_ERR("failed to set ADC filter setup 1 in start()");
      return;
    }*/

    ch_ready[0] = false;
    ch_ready[1] = false;
    batch_count = 0;

    adc->unmaskReadyInterrupt();

    _running = true;

    LOG_INF("Build C: ExG started — FS reg=%u, ODR=%u SPS combined, %.1f SPS/ch",
            fs_val,
            sample_rates.reg_vals[sample_rate_idx],  // same as fs_val, for clarity
            sample_rates.true_sample_rates[sample_rate_idx]);
}

void ExG::stop() {
    if (!_active) return;

    _running = false;

    adc->disableReadyInterrupt();

    // Put ADC in standby mode
    if (adc != nullptr) {
        adc->setAdcControl(AD7124::OperatingMode::STANDBY, AD7124::PowerMode::FULL_POWER, true);
    }

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);

    batch_count = 0;
    ch_ready[0] = false;
    ch_ready[1] = false;
    _active = false;

    LOG_INF("ExG sensor stopped");
}
