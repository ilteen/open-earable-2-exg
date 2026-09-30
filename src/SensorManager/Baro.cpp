#include "Baro.h"

#include "SensorManager.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMP388);

static const BaroConfig right_config = {
	.i2c_dev = &I2C3,
	.i2c_addr = DT_REG_ADDR(DT_NODELABEL(bmp388)),
	.sensor_id = ID_TEMP_BARO_RIGHT,
	.name = "right",
};

static const BaroConfig left_config = {
	.i2c_dev = &I2C2,
	.i2c_addr = DT_REG_ADDR(DT_NODELABEL(bmp388_left)),
	.sensor_id = ID_TEMP_BARO_LEFT,
	.name = "left",
};

Baro Baro::right(right_config);
Baro Baro::left(left_config);

Baro::Baro(const BaroConfig& config) : config(config) {}

// Initialisierung der SampleRateSettings für Baro (BMP3)
const SampleRateSetting<18> Baro::sample_rates = {
    { BMP3_ODR_0_001_HZ, BMP3_ODR_0_003_HZ, BMP3_ODR_0_006_HZ, BMP3_ODR_0_01_HZ, 
      BMP3_ODR_0_02_HZ, BMP3_ODR_0_05_HZ, BMP3_ODR_0_1_HZ, BMP3_ODR_0_2_HZ, 
      BMP3_ODR_0_39_HZ, BMP3_ODR_0_78_HZ, BMP3_ODR_1_5_HZ, BMP3_ODR_3_1_HZ, 
      BMP3_ODR_6_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_25_HZ, BMP3_ODR_50_HZ, 
      BMP3_ODR_100_HZ, BMP3_ODR_200_HZ },   // reg_vals

    { 0.001, 0.003, 0.006, 0.01, 0.02, 0.05, 0.1, 0.2, 
      0.39, 0.78, 1.5, 3.1, 6.25, 12.5, 25.0, 50.0, 
      100.0, 200.0 },  // sample_rates

    { 0.001, 0.003, 0.006, 0.01, 0.02, 0.05, 0.1, 0.2, 
      0.39, 0.78, 1.5, 3.1, 6.25, 12.5, 25.0, 50.0, 
      100.0, 200.0 }   // true_sample_rates
};

void Baro::do_update_sensor() {
	if (!bmp.performReading()) {
		LOG_WRN("Failed to read BMP388 sensor (%s)", config.name);
		return;
	}

	if (initial_discard > 0) {
		initial_discard--;
		return;
	}

	msg_baro.sd = _sd_logging;
	msg_baro.stream = _ble_stream;

	msg_baro.data.id = config.sensor_id;
	msg_baro.data.size = sizeof(float);
	msg_baro.data.time = micros();

	const float pressure = static_cast<float>(bmp.pressure);
	memcpy(msg_baro.data.data, &pressure, sizeof(pressure));

	int ret = k_msgq_put(sensor_queue, &msg_baro, K_NO_WAIT);
	if (ret) {
		LOG_WRN("sensor msg queue full (%s)", config.name);
	}
}

void Baro::update_right(struct k_work *work) {
	ARG_UNUSED(work);
	right.do_update_sensor();
}

void Baro::update_left(struct k_work *work) {
	ARG_UNUSED(work);
	left.do_update_sensor();
}

void Baro::right_timer_handler(struct k_timer *dummy) {
	ARG_UNUSED(dummy);
	k_work_submit_to_queue(&sensor_work_q, &right.sensor_work);
}

void Baro::left_timer_handler(struct k_timer *dummy) {
	ARG_UNUSED(dummy);
	k_work_submit_to_queue(&sensor_work_q, &left.sensor_work);
}

bool Baro::init(struct k_msgq * queue) {
	if (!_active) {
		pm_device_runtime_get(ls_1_8);
    	_active = true;
	}

	if (!bmp.begin_I2C(config.i2c_addr, config.i2c_dev)) {
		LOG_WRN("Could not find a valid BMP388 sensor (%s), check wiring!", config.name);
		pm_device_runtime_put(ls_1_8);
		_active = false;
		return false;
    }

	sensor_queue = queue;
	
	if (config.sensor_id == ID_TEMP_BARO_RIGHT) {
		k_work_init(&sensor_work, update_right);
		k_timer_init(&sensor_timer, right_timer_handler, NULL);
	} else {
		k_work_init(&sensor_work, update_left);
		k_timer_init(&sensor_timer, left_timer_handler, NULL);
	}

	return true;
}

void Baro::start(int sample_rate_idx) {
	initial_discard = 1;

    k_timeout_t t = K_USEC(1000000.0f / sample_rates.true_sample_rates[sample_rate_idx]);
    
    //bmp.set_interrogation_rate(setting.reg_val);
    //bmp.start();

	k_timer_start(&sensor_timer, K_NO_WAIT, t);

	_running = true;
}

void Baro::stop() {
	if (!_active) return;
    _active = false;

	_running = false;

	k_timer_stop(&sensor_timer);

    pm_device_runtime_put(ls_1_8);
}
