/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

//#include "../src/modules/sd_card.h"

#include <zephyr/settings/settings.h>

#include "macros_common.h"
#include "openearable_common.h"
#include "streamctrl.h"

#include "../src/Battery/PowerManager.h"
#include "../src/SensorManager/SensorManager.h"
#include "../src/utils/StateIndicator.h"

#include "device_info.h"
#include "battery_service.h"
#include "button_service.h"
#include "sensor_service.h"
#include "led_service.h"

#include "SensorScheme.h"
#include "DefaultSensors.h"

#include "time_sync.h"

#include "../src/SD_Card/SDLogger/SDLogger.h"

#include "uicr.h"

#include "streamctrl.h"

#include "bt_mgmt.h"

#include "bt_mgmt_conn_interval.h"
#include "conn_interval/conn_intvl_linear.h"

//#include "sd_card.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);
//BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
//	     "Console device is not ACM CDC UART device");

/* STEP 5.4 - Include header for USB */
#include <zephyr/usb/usb_device.h>

int main(void) {
    int ret;

    LOG_DBG("nRF5340 APP core started");

    ret = power_manager.begin();
    ERR_CHK(ret);

    uint8_t standalone = uicr_standalone_get();

    LOG_INF("Standalone mode: %i", standalone);

    /*sdcard_manager.init();

    sdcard_manager.mount();*/

    /* STEP 5.5 - Enable USB */
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        ret = usb_enable(NULL);
        if (ret) {
            LOG_ERR("Failed to enable USB");
            return 0;
        }
    }

    streamctrl_start();

    uint32_t sirk = uicr_sirk_get();

    if (sirk == 0xFFFFFFFFU) {
        state_indicator.set_pairing_state(SET_PAIRING);
    } else if (bonded_device_count > 0 && !oe_boot_state.timer_reset) {
        state_indicator.set_pairing_state(PAIRED);
    } else {
        state_indicator.set_pairing_state(BONDING);
    }

    init_sensor_manager();

    // sensor_config imu = {ID_IMU, 80, 0};
    // sensor_config imu = {ID_PPG_right_I2C2, 400, 0};
    // sensor_config temp = {ID_OPTTEMP, 10, 0};
    //  sensor_config temp = {ID_BONE_CONDUCTION, 100, 0};

    // config_sensor(&temp);

    // sensor_config ppg = {ID_PPG_right_I2C2, 400, 0};
    // config_sensor(&ppg);

    ret = init_led_service();
    ERR_CHK(ret);

    ret = init_battery_service();
    ERR_CHK(ret);

    ret = init_button_service();
    ERR_CHK(ret);

    ret = initParseInfoService(&defaultSensorIds, defaultSensors);
    ERR_CHK(ret);

    ret = init_sensor_service();
    ERR_CHK(ret);
    
    bt_mgmt_conn_interval_init(new ConnIntvlLinear(
                                                   4,                // linear increase step (8ms units)
                                                   CONFIG_BLE_ACL_CONN_INTERVAL,
                                                   CONFIG_BLE_ACL_CONN_INTERVAL_SLOW
                                                   ));
    
    // Register callback to start sensors after time sync
    time_sync_register_callback([]() {
        LOG_INF("Time synced - starting sensor recording");

        sensor_config exg = {
            .sensorId = ID_EXG,
            .sampleRateIndex = 3,  // 200 Hz
            .storageOptions = DATA_STORAGE};
        
        sensor_config imu = {
            .sensorId = ID_IMU,
            .sampleRateIndex = 3,  // 200 Hz
            .storageOptions = DATA_STORAGE};
        
        sensor_config ppg_right = {
            .sensorId = ID_PPG_right_I2C2,
            .sampleRateIndex = 4,  // 200 Hz
            .storageOptions = DATA_STORAGE};
        
        sensor_config ppg_left = {
            .sensorId = ID_PPG_left_I2C3,
            .sampleRateIndex = 4,  // 200 Hz
            .storageOptions = DATA_STORAGE};

        sensor_config temp_right = {
            .sensorId = ID_OPTTEMP_right_I2C2,
            .sampleRateIndex = 1,  // 1 Hz
            .storageOptions = DATA_STORAGE};
        
        sensor_config temp_left = {
            .sensorId = ID_OPTTEMP_left_I2C3,
            .sampleRateIndex = 1,  // 1 Hz
            .storageOptions = DATA_STORAGE};

        config_sensor(&temp_right);
        k_msleep(200);
        config_sensor(&temp_left);
        k_msleep(200);
        config_sensor(&imu);
        k_msleep(200);    
        config_sensor(&ppg_right);
        k_msleep(200);
        config_sensor(&ppg_left);
        k_msleep(200);
        config_sensor(&exg);

        LOG_INF("Sensors auto-started: ExG@200Hz, IMU@200Hz, PPG@200Hz, Temp@1Hz");
    });
    
    ret = init_time_sync();
    ERR_CHK(ret);

    LOG_INF("Waiting for time sync before starting sensor recording...");

    return 0;
}