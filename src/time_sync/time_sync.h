#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Callback type for time sync events.
 */
typedef void (*time_sync_callback_t)(void);

/**
 * @brief Get time since boot in microseconds.
 * @return Time since boot in microseconds.
 */
uint64_t get_time_since_boot_us(void);

/**
 * @brief Get the current synchronized time in microseconds (since 1. January 1970).
 * @return Current synchronized time in microseconds.
 */
uint64_t get_current_time_us(void);

/**
 * @brief Check whether the wall-clock time has been synchronized.
 * @return True after the Time Offset characteristic has been written.
 */
bool time_sync_is_synced(void);

/**
 * @brief Register a callback to be called when time is first synchronized.
 * @param callback Function to call when time is synced.
 */
void time_sync_register_callback(time_sync_callback_t callback);

/**
 * @brief Set the time offset (absolute value).
 * 
 * This is called by sync sources (USB, etc.) to set the offset directly.
 * Also triggers the first-sync callback if not yet synced.
 * 
 * @param offset_us The absolute time offset in microseconds.
 */
void time_sync_set_offset(int64_t offset_us);

/**
 * @brief Apply a delta to the time offset.
 * 
 * This is called by sync sources (BLE) to adjust the offset incrementally.
 * Also triggers the first-sync callback if not yet synced.
 * 
 * @param delta_us The delta to add to the current offset in microseconds.
 */
void time_sync_apply_offset(int64_t delta_us);

/**
 * @brief Persist the current synchronized UNIX time snapshot to settings.
 *
 * Intended for long-running scheduled waits so reset recovery can restore time
 * close to the latest known wall clock.
 */
void time_sync_persist_now_snapshot(void);

/**
 * @brief Initialize the time synchronization module.
 *
 * @return 0 on success, negative error code on failure.
 */
int init_time_sync();

#ifdef __cplusplus
}
#endif
