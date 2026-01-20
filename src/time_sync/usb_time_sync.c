/*
 * USB Time Sync Module for OpenEarable v2
 * 
 * Implements time synchronization over USB CDC serial connection.
 */

#include "usb_time_sync.h"
#include "time_sync.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_time_sync, LOG_LEVEL_INF);

// Protocol constants
#define USB_SYNC_MAGIC      0xAA // We're using the same CDC ACM device as mcumgr - this means time sync packets
#define USB_SYNC_REQUEST    0x01 // need to be distinguishable from mcumgr packets (they are, via magic byte)
#define USB_SYNC_RESPONSE   0x02
#define USB_SYNC_OFFSET     0x03

#define REQUEST_SIZE        11
#define RESPONSE_SIZE       27
#define OFFSET_SIZE         10

// Packet structures
struct __packed usb_sync_request {
    uint8_t magic;      // 0xAA
    uint8_t op;         // 0x01 = request
    uint8_t seq;        // Sequence number
    int64_t t1;         // Host send timestamp (microseconds)
};

struct __packed usb_sync_response {
    uint8_t magic;      // 0xAA
    uint8_t op;         // 0x02 = response
    uint8_t seq;        // Sequence number (echo)
    int64_t t1;         // Host send timestamp (echo)
    int64_t t2;         // Device receive timestamp
    int64_t t3;         // Device send timestamp
};

struct __packed usb_sync_offset {
    uint8_t magic;      // 0xAA
    uint8_t op;         // 0x03 = offset
    int64_t offset;     // Calculated time offset
};

// Module state
static const struct device *cdc_dev = NULL;
static bool usb_connected = false;

// Receive buffer and state
static uint8_t rx_buffer[64];
static size_t rx_pos = 0;

static void send_response(uint8_t seq, int64_t t1, int64_t t2, int64_t t3) {
    struct usb_sync_response resp = {
        .magic = USB_SYNC_MAGIC,
        .op = USB_SYNC_RESPONSE,
        .seq = seq,
        .t1 = t1,
        .t2 = t2,
        .t3 = t3
    };
    
    uint8_t *data = (uint8_t *)&resp;
    for (size_t i = 0; i < sizeof(resp); i++) {
        uart_poll_out(cdc_dev, data[i]);
    }
    
    LOG_DBG("USB sync response sent: seq=%d, t1=%lld, t2=%lld, t3=%lld", 
            seq, t1, t2, t3);
}

static void process_request(uint8_t *data, size_t len) {
    if (len < REQUEST_SIZE) {
        return;
    }
    
    int64_t t2 = get_time_since_boot_us();  // Device receive time
    
    struct usb_sync_request *req = (struct usb_sync_request *)data;
    
    int64_t t3 = get_time_since_boot_us();  // Device send time
    send_response(req->seq, req->t1, t2, t3);
    
    LOG_DBG("USB sync request: seq=%d, t1=%lld", req->seq, req->t1);
}

static void process_offset(uint8_t *data, size_t len) {
    if (len < OFFSET_SIZE) {
        return;
    }
    
    struct usb_sync_offset *pkt = (struct usb_sync_offset *)data;
    
    LOG_INF("USB time synced! Offset: %lld us (%.6f s)", 
            pkt->offset, (double)pkt->offset / 1000000.0);
    
    // Set offset via time_sync module (also triggers first-sync callback)
    time_sync_set_offset(pkt->offset);
}

static void process_packet(uint8_t *data, size_t len) {
    if (len < 2 || data[0] != USB_SYNC_MAGIC) {
        LOG_WRN("Invalid USB sync packet: magic=0x%02X, len=%zu", 
                len > 0 ? data[0] : 0, len);
        return;
    }
    
    uint8_t op = data[1];
    
    switch (op) {
        case USB_SYNC_REQUEST:
            process_request(data, len);
            break;
        case USB_SYNC_OFFSET:
            process_offset(data, len);
            break;
        default:
            LOG_WRN("Unknown USB sync op: 0x%02X", op);
            break;
    }
}

static void uart_irq_handler(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    
    if (!uart_irq_update(dev)) {
        return;
    }
    
    while (uart_irq_rx_ready(dev)) {
        uint8_t c;
        int ret = uart_fifo_read(dev, &c, 1);
        if (ret != 1) {
            break;
        }
        
        // Wait for magic byte to start packet
        if (rx_pos == 0 && c != USB_SYNC_MAGIC) {
            continue;
        }
        
        rx_buffer[rx_pos++] = c;
        
        // Check if we have enough to determine packet type and length
        if (rx_pos >= 2) {
            size_t expected_len = 0;
            switch (rx_buffer[1]) {
                case USB_SYNC_REQUEST:
                    expected_len = REQUEST_SIZE;
                    break;
                case USB_SYNC_OFFSET:
                    expected_len = OFFSET_SIZE;
                    break;
                default:
                    // Unknown packet type, reset
                    LOG_WRN("Unknown packet op 0x%02X, resetting", rx_buffer[1]);
                    rx_pos = 0;
                    continue;
            }
            
            if (rx_pos >= expected_len) {
                process_packet(rx_buffer, rx_pos);
                rx_pos = 0;
            }
        }
        
        // Prevent buffer overflow
        if (rx_pos >= sizeof(rx_buffer)) {
            LOG_WRN("USB sync rx buffer overflow, resetting");
            rx_pos = 0;
        }
    }
}

int usb_time_sync_init(void) {
    // Get CDC ACM UART device
    // Note: Using the same CDC ACM device as mcumgr - this means time sync
    // packets need to be distinguishable from mcumgr packets (they are, via magic byte)
    cdc_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
    
    if (!device_is_ready(cdc_dev)) {
        LOG_WRN("USB CDC device not ready - USB time sync disabled");
        return 0;  // Not an error, USB might not be available
    }
    
    // Set up UART interrupt handler
    uart_irq_callback_user_data_set(cdc_dev, uart_irq_handler, NULL);
    uart_irq_rx_enable(cdc_dev);
    
    usb_connected = true;
    LOG_INF("USB time sync initialized on %s", cdc_dev->name);
    
    return 0;
}

bool usb_time_sync_is_connected(void) {
    return usb_connected && device_is_ready(cdc_dev);
}
