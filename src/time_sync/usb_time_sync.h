/*
 * USB Time Sync Module for OpenEarable v2
 * 
 * Enables time synchronization over USB CDC serial connection.
 * This provides lower latency and more accurate sync than BLE.
 * 
 * Protocol:
 *   Request:  [0xAA][0x01][seq:1][t1:8]           = 11 bytes
 *   Response: [0xAA][0x02][seq:1][t1:8][t2:8][t3:8] = 27 bytes
 *   Offset:   [0xAA][0x03][offset:8]              = 10 bytes
 *   RecName:  [0xAA][0x04][name_len:1][name:n]
 *   Schedule: [0xAA][0x05][sensor:1][sr_idx:1][storage:1][res:1][start_us:8] = 14 bytes
 *   Ack:      [0xAA][0x06][cmd:1][status:1]       = 4 bytes
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the USB time synchronization module.
 *
 * Sets up the USB CDC serial interrupt handler to listen for
 * time sync packets.
 *
 * @return 0 on success, negative error code on failure.
 */
int usb_time_sync_init(void);

/**
 * @brief Check if USB CDC is connected and ready.
 *
 * @return true if USB is connected, false otherwise.
 */
bool usb_time_sync_is_connected(void);

#ifdef __cplusplus
}
#endif
