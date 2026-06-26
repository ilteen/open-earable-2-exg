#!/usr/bin/env python3
"""
GUI tool for OpenEarable ExG:
- Sync time via BLE or USB
- Import .oe files and export to CSV

Features:
- Scan and display OpenEarable devices (BLE)
- Scan and display USB serial ports
- Select device and sync time via BLE or USB
- Debug console for USB communication
- Import .oe sensor recordings
- Export to CSV format
"""

# Debug mode - set to True to show debug console in USB tab
DEBUG = False

import asyncio
import os
import re
import struct
import sys
import threading
import time
from collections import deque
from collections import defaultdict
from typing import Optional
from typing import Callable
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for embedding
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from io import BytesIO
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
import digitalfilter

# Try PyQt6 first, fall back to PySide6 (better Windows compatibility)
try:
    from PyQt6.QtCore import Qt, QTimer, QDateTime, pyqtSignal, QObject, QByteArray
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QLabel, QTableWidget, QTableWidgetItem, QProgressBar,
        QMessageBox, QHeaderView, QGroupBox, QTabWidget, QFileDialog,
        QListWidget, QTextEdit, QCheckBox, QScrollArea, QSplitter,
        QComboBox, QStackedWidget,
        QLineEdit, QDateTimeEdit, QFormLayout
    )
    from PyQt6.QtGui import QTextCursor, QFont, QPixmap
    QT_BACKEND = "PyQt6"
except ImportError:
    try:
        from PySide6.QtCore import Qt, QTimer, QDateTime, Signal as pyqtSignal, QObject, QByteArray
        from PySide6.QtWidgets import (
            QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
            QPushButton, QLabel, QTableWidget, QTableWidgetItem, QProgressBar,
            QMessageBox, QHeaderView, QGroupBox, QTabWidget, QFileDialog,
            QListWidget, QTextEdit, QCheckBox, QScrollArea, QSplitter,
            QComboBox, QStackedWidget,
            QLineEdit, QDateTimeEdit, QFormLayout
        )
        from PySide6.QtGui import QTextCursor, QFont, QPixmap
        QT_BACKEND = "PySide6"
    except ImportError:
        print("Error: Neither PyQt6 nor PySide6 is installed.")
        print("Please install one of them:")
        print("  pip install PyQt6")
        print("  pip install PySide6  (recommended for Windows)")
        sys.exit(1)

# Try to import serial, but make it optional
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. USB sync will be disabled.")


# =============================================================================
# BLE Time Sync
# =============================================================================

TIME_SYNC_SERVICE_UUID = "2e04cbf7-939d-4be5-823e-271838b75259"
TIME_SYNC_OFFSET_CHAR_UUID = "2e04cbf8-939d-4be5-823e-271838b75259"
TIME_SYNC_RTT_CHAR_UUID = "2e04cbf9-939d-4be5-823e-271838b75259"

PACKET_FORMAT = "<BBHqqq"
PACKET_SIZE = 28
TIME_SYNC_SAMPLES = 7
RUN_NOW_LEAD_TIME_US = 12_000_000
BLE_SCAN_TIMEOUT_SECONDS = 30.0
BLE_SCAN_CHUNK_SECONDS = 1.0
BLE_CONNECT_ATTEMPTS = 3
BLE_CONNECT_RETRY_DELAY_S = 2.0
APP_LAUNCH_UNIX_TS = int(time.time())
DEFAULT_PARTICIPANT_ID = f"P{APP_LAUNCH_UNIX_TS % 10000:04d}"
# ExG sample-rate index mapping is defined in src/SensorManager/ExG.cpp.
# Current mapping uses index 5 for 256 Hz.
EXG_SAMPLERATE_INDEX_256HZ = 5

SENSOR_SERVICE_UUID = "34c2e3bb-34aa-11eb-adc1-0242ac120002"
SENSOR_CONFIG_CHAR_UUID = "34c2e3be-34aa-11eb-adc1-0242ac120002"
SENSOR_DATA_CHAR_UUID = "34c2e3bc-34aa-11eb-adc1-0242ac120002"
SENSOR_RECORDING_NAME_CHAR_UUID = "34c2e3c0-34aa-11eb-adc1-0242ac120002"
SENSOR_SCHEDULED_START_CHAR_UUID = "34c2e3c1-34aa-11eb-adc1-0242ac120002"

SENSOR_ID_EXG = 9
DATA_STREAMING = 0x01
DATA_STORAGE = 0x02

SENSOR_CONFIG_FORMAT = "<BBB"
SCHEDULED_START_FORMAT = "<BBBBQ"
SENSOR_PACKET_HEADER_FORMAT = "<BBQ"
SENSOR_PACKET_HEADER_SIZE = struct.calcsize(SENSOR_PACKET_HEADER_FORMAT)

MAX_RECORDING_NAME_LEN = 63


def build_sensor_config(sensor_id: int, sample_rate_index: int, storage_options: int) -> bytes:
    return struct.pack(SENSOR_CONFIG_FORMAT, sensor_id, sample_rate_index, storage_options)


def sanitize_participant_id(raw_participant_id: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]", "_", raw_participant_id.strip())
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")
    return cleaned or "participant"


def build_recording_name_prefix(participant_id: str, scheduled_start_us: int) -> str:
    participant = sanitize_participant_id(participant_id)
    local_time = time.localtime(scheduled_start_us / 1_000_000)
    start_token = time.strftime("%d-%m-%Y_%H-%M", local_time)
    filename = f"{participant}_{start_token}"
    return filename[:MAX_RECORDING_NAME_LEN]


def build_scheduled_start_packet(
    sensor_id: int,
    sample_rate_index: int,
    storage_options: int,
    scheduled_start_us: int,
) -> bytes:
    return struct.pack(
        SCHEDULED_START_FORMAT,
        sensor_id,
        sample_rate_index,
        storage_options,
        0,
        scheduled_start_us,
    )


def parse_sensor_packet(data: bytes) -> Optional[tuple[int, int, int, bytes]]:
    if len(data) < SENSOR_PACKET_HEADER_SIZE:
        return None
    sensor_id, payload_size, timestamp_us = struct.unpack(
        SENSOR_PACKET_HEADER_FORMAT, data[:SENSOR_PACKET_HEADER_SIZE]
    )
    end = SENSOR_PACKET_HEADER_SIZE + payload_size
    if end > len(data):
        return None
    return sensor_id, payload_size, timestamp_us, data[SENSOR_PACKET_HEADER_SIZE:end]


def create_request_packet(seq: int, t1_us: int) -> bytes:
    return struct.pack(PACKET_FORMAT, 1, 0, seq, t1_us, 0, 0)


def parse_response_packet(data: bytes) -> Optional[dict]:
    if len(data) < PACKET_SIZE:
        return None
    version, op, seq, t1, t2, t3 = struct.unpack(PACKET_FORMAT, data[:PACKET_SIZE])
    if op != 1:
        return None
    return {"t1_phone_send": t1, "t2_device_rx": t2, "t3_device_tx": t3}


def compute_median(values: list[int]) -> int:
    sorted_vals = sorted(values)
    mid = len(sorted_vals) // 2
    if len(sorted_vals) % 2 == 1:
        return sorted_vals[mid]
    return (sorted_vals[mid - 1] + sorted_vals[mid]) // 2


def get_ble_rssi(device: BLEDevice) -> Optional[int]:
    """Return RSSI if available on this Bleak backend/device object."""
    rssi = getattr(device, "rssi", None)
    if rssi is None:
        details = getattr(device, "details", None)
        rssi = getattr(details, "rssi", None) if details is not None else None
    try:
        return int(rssi) if rssi is not None else None
    except (TypeError, ValueError):
        return None


def ble_sort_key(device: BLEDevice) -> int:
    rssi = get_ble_rssi(device)
    return -(rssi if rssi is not None else -100)


async def scan_devices(
    timeout: float = 5.0,
    progress_callback=None,
    should_stop=None,
) -> tuple[list[BLEDevice], int]:
    """Scan for OpenEarable BLE devices.
    
    On macOS, this requires Bluetooth permission. If running as a bundled .app,
    the Info.plist must include NSBluetoothAlwaysUsageDescription.
    
    Scans for OpenEarable BLE devices.
    
    Returns:
        (list of OpenEarable devices, total devices scanned)
    """
    try:
        devices_found = {}
        seen_addresses: set[str] = set()
        elapsed = 0.0

        while elapsed < timeout:
            if should_stop and should_stop():
                break

            chunk_timeout = min(BLE_SCAN_CHUNK_SECONDS, timeout - elapsed)
            discovered = await BleakScanner.discover(timeout=chunk_timeout, return_adv=True)
            elapsed += chunk_timeout

            for address, (device, advertisement_data) in discovered.items():
                seen_addresses.add(address)

                name = device.name or advertisement_data.local_name
                service_uuids = [str(s).lower() for s in (advertisement_data.service_uuids or [])]
                name_l = (name or "").lower()
                is_openearable = (
                    ("openearable" in name_l)
                    or ("open-earable" in name_l)
                    or ("open earable" in name_l)
                    or TIME_SYNC_SERVICE_UUID.lower() in service_uuids
                    or SENSOR_SERVICE_UUID.lower() in service_uuids
                )
                if is_openearable:
                    devices_found[address] = device

            if progress_callback:
                partial_devices = sorted(list(devices_found.values()), key=ble_sort_key)
                progress_callback(partial_devices, len(seen_addresses), elapsed, timeout)

            if should_stop and should_stop():
                break

        devices_list = sorted(list(devices_found.values()), key=ble_sort_key)
        return devices_list, len(seen_addresses)
    except Exception as e:
        # Re-raise with more context for debugging
        raise RuntimeError(f"BLE scan failed: {e}. Make sure Bluetooth is enabled and the app has permission to use it.") from e


async def sync_time(device: BLEDevice | str, progress_callback=None) -> tuple[bool, str]:
    device_addr = device.address if isinstance(device, BLEDevice) else str(device)
    device_name = device.name if isinstance(device, BLEDevice) else device_addr
    client_target = device if isinstance(device, BLEDevice) else device_addr
    try:
        async with BleakClient(client_target) as client:
            if progress_callback:
                progress_callback(f"Connected to {device_name}")
            success, message, _ = await sync_time_with_client(client, progress_callback)
            return success, message
    except Exception as e:
        return False, f"Error: {str(e)}"


async def ble_write_with_retry(
    client: BleakClient,
    char_uuid: str,
    data: bytes,
    *,
    response: bool,
    attempts: int = 3,
    delay_s: float = 0.25,
) -> None:
    last_err = None
    for attempt in range(attempts):
        try:
            await client.write_gatt_char(char_uuid, data, response=response)
            return
        except Exception as e:
            last_err = e
            if attempt < attempts - 1:
                await asyncio.sleep(delay_s)
    raise last_err


async def ble_read_with_retry(
    client: BleakClient,
    char_uuid: str,
    *,
    attempts: int = 3,
    delay_s: float = 0.25,
) -> bytes:
    last_err = None
    for attempt in range(attempts):
        try:
            return bytes(await client.read_gatt_char(char_uuid))
        except Exception as e:
            last_err = e
            if attempt < attempts - 1:
                await asyncio.sleep(delay_s)
    raise last_err


async def sync_time_with_client(client: BleakClient, progress_callback=None) -> tuple[bool, str, int]:
    offsets: list[int] = []
    response_event = asyncio.Event()
    current_response: dict = {}

    def notification_handler(sender, data: bytearray):
        nonlocal current_response
        t4 = int(time.time() * 1_000_000)
        pkt = parse_response_packet(bytes(data))
        if pkt is None:
            return
        t1 = pkt["t1_phone_send"]
        t3 = pkt["t3_device_tx"]
        unix_at_t3 = t1 + ((t4 - t1) // 2)
        offset = unix_at_t3 - t3
        current_response = {"offset": offset}
        response_event.set()

    await client.start_notify(TIME_SYNC_RTT_CHAR_UUID, notification_handler)
    try:
        for i in range(TIME_SYNC_SAMPLES):
            if progress_callback:
                progress_callback(f"Collecting sample {i + 1}/{TIME_SYNC_SAMPLES}...")

            t1 = int(time.time() * 1_000_000)
            request = create_request_packet(i, t1)
            response_event.clear()
            await client.write_gatt_char(TIME_SYNC_RTT_CHAR_UUID, request)

            try:
                await asyncio.wait_for(response_event.wait(), timeout=2.0)
                offsets.append(current_response["offset"])
            except asyncio.TimeoutError:
                pass

            await asyncio.sleep(0.05)
    finally:
        await client.stop_notify(TIME_SYNC_RTT_CHAR_UUID)

    if len(offsets) < 1:
        return False, "No valid samples collected!", 0

    median_offset = compute_median(offsets)
    offset_bytes = struct.pack("<q", median_offset)
    await ble_write_with_retry(client, TIME_SYNC_OFFSET_CHAR_UUID, offset_bytes, response=True)

    offset_sec = median_offset / 1_000_000
    message = (
        f"Time synced!\n\n"
        f"Offset: {offset_sec:+.3f} seconds\n"
        f"Samples: {len(offsets)}/{TIME_SYNC_SAMPLES}"
    )
    return True, message, median_offset


async def sync_time_and_schedule_exg_recording(
    device: BLEDevice | str,
    participant_id: str,
    scheduled_start_us: int,
    sample_rate_index: int = EXG_SAMPLERATE_INDEX_256HZ,
    device_display_name: Optional[str] = None,
    progress_callback=None,
) -> tuple[bool, str]:
    device_addr = device.address if isinstance(device, BLEDevice) else str(device)
    device_name = device_display_name or (device.name if isinstance(device, BLEDevice) else device_addr)
    client_target = device if isinstance(device, BLEDevice) else device_addr
    recording_prefix = build_recording_name_prefix(participant_id, scheduled_start_us)

    last_error: Optional[Exception] = None
    for attempt in range(BLE_CONNECT_ATTEMPTS):
        try:
            async with BleakClient(client_target) as client:
                if progress_callback:
                    progress_callback(f"Connected to {device_name}")

                success, sync_message, _ = await sync_time_with_client(client, progress_callback)
                if not success:
                    return False, sync_message

                await ble_write_with_retry(
                    client,
                    SENSOR_RECORDING_NAME_CHAR_UUID,
                    recording_prefix.encode("utf-8"),
                    response=True,
                )

                try:
                    configured_name = (
                        await ble_read_with_retry(client, SENSOR_RECORDING_NAME_CHAR_UUID)
                    ).decode("utf-8", errors="ignore").strip("\x00")
                except Exception as e:
                    return False, f"Failed to read back recording name after write: {e}"

                if configured_name != recording_prefix:
                    return (
                        False,
                        "Recording name verification failed.\n"
                        f"Expected: {recording_prefix}\n"
                        f"Device returned: {configured_name}",
                    )

                if progress_callback:
                    progress_callback("Recording name configured on device.")

                schedule_packet = build_scheduled_start_packet(
                    SENSOR_ID_EXG,
                    sample_rate_index,
                    DATA_STORAGE,
                    scheduled_start_us,
                )
                await ble_write_with_retry(
                    client,
                    SENSOR_SCHEDULED_START_CHAR_UUID,
                    schedule_packet,
                    response=True,
                )

                return (
                    True,
                    f"{sync_message}\n\n"
                    f"Scheduled EXG recording configured on device.\n"
                    f"Device can now be disconnected.\n"
                    f"Start time: {time.strftime('%d. %b %Y, %H:%M:%S', time.localtime(scheduled_start_us / 1_000_000))}\n"
                    f"Filename: {recording_prefix}.oe",
                )
        except Exception as e:
            last_error = e
            if attempt < BLE_CONNECT_ATTEMPTS - 1:
                if progress_callback:
                    progress_callback(
                        f"Connection failed ({attempt + 1}/{BLE_CONNECT_ATTEMPTS}), retrying..."
                    )
                await asyncio.sleep(BLE_CONNECT_RETRY_DELAY_S)

    msg = str(last_error) if last_error else "unknown BLE error"
    if "Peer failed to respond to ATT value indication" in msg:
        msg += (
            "\nConnection became unstable during GATT indication handling. "
            "Retries were attempted automatically."
        )
    return False, f"Error: {msg}"


# =============================================================================
# USB Time Sync
# =============================================================================

# USB Protocol constants (must match firmware usb_time_sync.c)
USB_SYNC_MAGIC = 0xAA
USB_SYNC_REQUEST = 0x01
USB_SYNC_RESPONSE = 0x02
USB_SYNC_OFFSET = 0x03
USB_SYNC_SET_RECORDING_NAME = 0x04
USB_SYNC_SCHEDULED_START = 0x05
USB_SYNC_ACK = 0x06

USB_REQUEST_SIZE = 11   # magic(1) + op(1) + seq(1) + t1(8)
USB_RESPONSE_SIZE = 27  # magic(1) + op(1) + seq(1) + t1(8) + t2(8) + t3(8)
USB_OFFSET_SIZE = 10    # magic(1) + op(1) + offset(8)
USB_ACK_SIZE = 4        # magic(1) + op(1) + cmd(1) + status(1)

USB_TIME_SYNC_SAMPLES = 10  # More samples for USB since it's faster


def find_openearable_usb_ports() -> list[tuple[str, str, str]]:
    """Find USB serial ports that are OpenEarable devices.
    
    Returns list of (port, description, hwid) tuples.
    """
    if not SERIAL_AVAILABLE:
        return []
    
    ports = []
    for port in serial.tools.list_ports.comports():
        description = port.description or ""
        product = port.product or ""
        manufacturer = port.manufacturer or ""
        hwid = port.hwid or ""
        display_name = product if product else description
        
        text = f"{description} {product} {manufacturer} {hwid}".lower()
        is_openearable = (
            "openearable" in text
        )

        if is_openearable:
            ports.append((port.device, display_name, hwid))
    return ports


def create_usb_request_packet(seq: int, t1_us: int) -> bytes:
    """Create USB time sync request packet."""
    return struct.pack('<BBBq', USB_SYNC_MAGIC, USB_SYNC_REQUEST, seq, t1_us)


def create_usb_offset_packet(offset_us: int) -> bytes:
    """Create USB time sync offset packet."""
    return struct.pack('<BBq', USB_SYNC_MAGIC, USB_SYNC_OFFSET, offset_us)


def create_usb_recording_name_packet(recording_name: str) -> bytes:
    """Create USB packet to set recording filename prefix."""
    encoded = recording_name.encode("utf-8")[:MAX_RECORDING_NAME_LEN]
    return bytes([USB_SYNC_MAGIC, USB_SYNC_SET_RECORDING_NAME, len(encoded)]) + encoded


def create_usb_scheduled_start_packet(
    sensor_id: int,
    sample_rate_index: int,
    storage_options: int,
    scheduled_start_us: int,
) -> bytes:
    """Create USB packet to schedule sensor start."""
    return struct.pack(
        '<BBBBBBQ',
        USB_SYNC_MAGIC,
        USB_SYNC_SCHEDULED_START,
        sensor_id,
        sample_rate_index,
        storage_options,
        0,
        scheduled_start_us,
    )


def parse_usb_response_packet(data: bytes) -> Optional[dict]:
    """Parse USB time sync response packet."""
    if len(data) < USB_RESPONSE_SIZE:
        return None
    
    magic, op, seq = struct.unpack('<BBB', data[:3])
    if magic != USB_SYNC_MAGIC or op != USB_SYNC_RESPONSE:
        return None
    
    t1, t2, t3 = struct.unpack('<qqq', data[3:27])
    return {
        "seq": seq,
        "t1": t1,  # Host send time (echoed back)
        "t2": t2,  # Device receive time
        "t3": t3,  # Device send time
    }


def parse_usb_ack_packet(data: bytes) -> Optional[dict]:
    """Parse USB ack packet."""
    if len(data) < USB_ACK_SIZE:
        return None
    magic, op, cmd, status = struct.unpack('<BBBB', data[:USB_ACK_SIZE])
    if magic != USB_SYNC_MAGIC or op != USB_SYNC_ACK:
        return None
    return {"cmd": cmd, "status": status}


def format_usb_status_error(expected_cmd: int, status: int) -> str:
    details = {
        22: "invalid parameters (check sensor id/storage/name)",
        34: "start time out of range (device time not synced yet or start is too close/far)",
    }
    detail = details.get(status)
    if detail:
        return f"Device rejected command 0x{expected_cmd:02X} (status={status}: {detail})"
    return f"Device rejected command 0x{expected_cmd:02X} (status={status})"


def _sync_time_usb_on_serial(ser, progress_callback=None, debug_callback=None) -> tuple[bool, str, int]:
    """Perform USB time sync using an already-open serial connection."""
    offsets: list[int] = []
    rtts: list[int] = []

    def debug(msg: str):
        if debug_callback:
            debug_callback(msg)

    # Clear pending data before sampling
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)
    debug("Buffers cleared, starting sync samples...")

    for i in range(USB_TIME_SYNC_SAMPLES):
        if progress_callback:
            progress_callback(f"Collecting sample {i + 1}/{USB_TIME_SYNC_SAMPLES}...")

        t1 = int(time.time() * 1_000_000)
        request = create_usb_request_packet(i, t1)
        debug(f"\n[TX] Sample {i+1}: magic=0xAA op=0x01 seq={i} t1={t1}")
        debug(f"     Raw: {request.hex()}")

        ser.write(request)
        ser.flush()

        response = ser.read(USB_RESPONSE_SIZE)
        t4 = int(time.time() * 1_000_000)

        if len(response) == USB_RESPONSE_SIZE:
            debug(f"[RX] Got {len(response)} bytes: {response.hex()}")

            pkt = parse_usb_response_packet(response)
            if pkt:
                t2 = pkt["t2"]
                t3 = pkt["t3"]

                # Device timestamps (t2/t3) are in "time since boot",
                # while host timestamps (t1/t4) are Unix epoch time.
                # Estimate Unix time at device t3 and derive absolute offset.
                rtt = t4 - t1
                unix_at_t3 = t1 + (rtt // 2)
                offset = unix_at_t3 - t3

                debug(f"     t1={t1}, t2={t2}, t3={t3}, t4={t4}")
                debug(f"     RTT={rtt}µs ({rtt/1000:.3f}ms), offset={offset}µs")

                offsets.append(offset)
                rtts.append(rtt)
            else:
                debug("     Failed to parse response!")
        else:
            debug(f"[RX] Timeout or incomplete: got {len(response)} bytes (expected {USB_RESPONSE_SIZE})")
            if response:
                debug(f"     Raw: {response.hex()}")

        time.sleep(0.02)

    if len(offsets) < 1:
        return False, "No valid samples collected!", 0

    median_offset = compute_median(offsets)
    avg_rtt = sum(rtts) / len(rtts)

    debug("\n=== Results ===")
    debug(f"Valid samples: {len(offsets)}/{USB_TIME_SYNC_SAMPLES}")
    debug(f"Offsets: {offsets}")
    debug(f"Median offset: {median_offset}µs ({median_offset/1_000_000:.6f}s)")
    debug(f"Average RTT: {avg_rtt:.0f}µs ({avg_rtt/1000:.3f}ms)")

    offset_packet = create_usb_offset_packet(median_offset)
    debug(f"\n[TX] Sending offset: {median_offset}µs")
    debug(f"     Raw: {offset_packet.hex()}")
    ser.write(offset_packet)
    ser.flush()
    ok, err = _wait_for_usb_ack(ser, USB_SYNC_OFFSET, debug_callback=debug_callback)
    if not ok:
        return False, f"USB time sync failed while applying offset: {err}", 0

    offset_sec = median_offset / 1_000_000
    rtt_ms = avg_rtt / 1000
    message = (
        f"Time synced via USB!\n\n"
        f"Offset: {offset_sec:+.6f} seconds\n"
        f"Average RTT: {rtt_ms:.3f} ms\n"
        f"Samples: {len(offsets)}/{USB_TIME_SYNC_SAMPLES}"
    )
    return True, message, median_offset


def _send_usb_command_with_ack(
    ser,
    packet: bytes,
    expected_cmd: int,
    debug_callback=None,
    ack_timeout: float = 2.0,
) -> tuple[bool, str]:
    """Send a USB command packet and wait for firmware ACK."""

    def debug(msg: str):
        if debug_callback:
            debug_callback(msg)

    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    debug(f"[TX] Command op=0x{expected_cmd:02X} raw={packet.hex()}")

    old_timeout = ser.timeout
    try:
        ser.timeout = ack_timeout
        ack_raw = ser.read(USB_ACK_SIZE)
    finally:
        ser.timeout = old_timeout

    if len(ack_raw) != USB_ACK_SIZE:
        return False, f"No ACK received for command 0x{expected_cmd:02X}"

    debug(f"[RX] ACK raw={ack_raw.hex()}")
    ack = parse_usb_ack_packet(ack_raw)
    if ack is None:
        return False, f"Invalid ACK for command 0x{expected_cmd:02X}"
    if ack["cmd"] != expected_cmd:
        return False, (
            f"Unexpected ACK command 0x{ack['cmd']:02X} "
            f"(expected 0x{expected_cmd:02X})"
        )
    if ack["status"] != 0:
        return False, format_usb_status_error(expected_cmd, ack["status"])
    return True, ""


def _wait_for_usb_ack(
    ser,
    expected_cmd: int,
    debug_callback=None,
    ack_timeout: float = 2.0,
) -> tuple[bool, str]:
    """Wait for a USB ACK packet for a previously sent command."""

    def debug(msg: str):
        if debug_callback:
            debug_callback(msg)

    old_timeout = ser.timeout
    try:
        ser.timeout = ack_timeout
        ack_raw = ser.read(USB_ACK_SIZE)
    finally:
        ser.timeout = old_timeout

    if len(ack_raw) != USB_ACK_SIZE:
        return False, f"No ACK received for command 0x{expected_cmd:02X}"

    debug(f"[RX] ACK raw={ack_raw.hex()}")
    ack = parse_usb_ack_packet(ack_raw)
    if ack is None:
        return False, f"Invalid ACK for command 0x{expected_cmd:02X}"
    if ack["cmd"] != expected_cmd:
        return False, (
            f"Unexpected ACK command 0x{ack['cmd']:02X} "
            f"(expected 0x{expected_cmd:02X})"
        )
    if ack["status"] != 0:
        return False, format_usb_status_error(expected_cmd, ack["status"])
    return True, ""


def sync_time_usb(port: str, progress_callback=None, debug_callback=None) -> tuple[bool, str]:
    """
    Perform USB time sync only.

    Args:
        port: Serial port path (e.g., /dev/tty.usbmodem*)
        progress_callback: Called with progress messages
        debug_callback: Called with debug info (tx/rx bytes, timing)

    Returns:
        (success, message) tuple
    """
    if not SERIAL_AVAILABLE:
        return False, "pyserial not installed. Run: pip install pyserial"

    def debug(msg: str):
        if debug_callback:
            debug_callback(msg)

    try:
        debug(f"Opening serial port: {port}")
        with serial.Serial(port, 115200, timeout=2.0) as ser:
            if progress_callback:
                progress_callback(f"Connected to {port}")
            debug("Serial port opened successfully")
            success, message, _ = _sync_time_usb_on_serial(ser, progress_callback, debug_callback)
            return success, message
    except serial.SerialException as e:
        debug(f"Serial error: {e}")
        return False, f"Serial error: {str(e)}"
    except Exception as e:
        debug(f"Error: {e}")
        return False, f"Error: {str(e)}"


def sync_time_and_schedule_exg_recording_usb(
    port: str,
    participant_id: str,
    scheduled_start_us: int,
    sample_rate_index: int = EXG_SAMPLERATE_INDEX_256HZ,
    progress_callback=None,
    debug_callback=None,
) -> tuple[bool, str]:
    """Sync time via USB, then configure EXG recording name and scheduled start."""
    if not SERIAL_AVAILABLE:
        return False, "pyserial not installed. Run: pip install pyserial"

    recording_prefix = build_recording_name_prefix(participant_id, scheduled_start_us)

    def debug(msg: str):
        if debug_callback:
            debug_callback(msg)

    try:
        debug(f"Opening serial port: {port}")
        with serial.Serial(port, 115200, timeout=2.0) as ser:
            if progress_callback:
                progress_callback(f"Connected to {port}")
            debug("Serial port opened successfully")

            success, sync_message, _ = _sync_time_usb_on_serial(ser, progress_callback, debug_callback)
            if not success:
                return False, sync_message

            if progress_callback:
                progress_callback("Writing recording name...")
            rec_name_packet = create_usb_recording_name_packet(recording_prefix)
            ok, err = _send_usb_command_with_ack(
                ser,
                rec_name_packet,
                USB_SYNC_SET_RECORDING_NAME,
                debug_callback=debug_callback,
            )
            if not ok:
                return False, f"{sync_message}\n\nFailed to set recording name: {err}"

            if progress_callback:
                progress_callback("Writing scheduled start...")
            schedule_packet = create_usb_scheduled_start_packet(
                SENSOR_ID_EXG,
                sample_rate_index,
                DATA_STORAGE,
                scheduled_start_us,
            )
            ok, err = _send_usb_command_with_ack(
                ser,
                schedule_packet,
                USB_SYNC_SCHEDULED_START,
                debug_callback=debug_callback,
            )
            if not ok:
                return False, f"{sync_message}\n\nFailed to set scheduled start: {err}"

            return (
                True,
                f"{sync_message}\n\n"
                f"Scheduled EXG recording configured via USB.\n"
                f"Device can now be disconnected.\n"
                f"Start time: {time.strftime('%d. %b %Y, %H:%M:%S', time.localtime(scheduled_start_us / 1_000_000))}\n"
                f"Filename: {recording_prefix}.oe",
            )
    except serial.SerialException as e:
        debug(f"Serial error: {e}")
        return False, f"Serial error: {str(e)}"
    except Exception as e:
        debug(f"Error: {e}")
        return False, f"Error: {str(e)}"


# =============================================================================
# .oe File Parsing (from notebook)
# =============================================================================

LABELS = {
    "imu": ['acc.x', 'acc.y', 'acc.z', 'gyro.x', 'gyro.y', 'gyro.z', 'mag.x', 'mag.y', 'mag.z'],
    "ppg_right": ['ppg_right.red', 'ppg_right.ir', 'ppg_right.green', 'ppg_right.ambient'],
    "ppg_left": ['ppg_left.red', 'ppg_left.ir', 'ppg_left.green', 'ppg_left.ambient'],
    "optical_temp_right": ['optical_temp_right.temperature'],
    "optical_temp_left": ['optical_temp_left.temperature'],
    "exg": ['exg.voltage'],
}

SENSOR_SID = {
    "imu": 0,
    "barometer": 1,
    "microphone": 2,
    "ppg_right": 4,
    "ppg_left": 5,
    "bone_acc": 6,
    "optical_temp_right": 7,
    "optical_temp_left": 8,
    "exg": 9,
}

SENSOR_FORMATS = {
    SENSOR_SID["imu"]: '<9f',
    SENSOR_SID["barometer"]: '<2f',
    SENSOR_SID["ppg_right"]: '<4I',
    SENSOR_SID["ppg_left"]: '<4I',
    SENSOR_SID["bone_acc"]: '<3h',
    SENSOR_SID["optical_temp_right"]: '<f',
    SENSOR_SID["optical_temp_left"]: '<f',
    SENSOR_SID["exg"]: '<f',
}

ACTIVE_SENSORS = ["imu", "ppg_right", "ppg_left", "optical_temp_right", "optical_temp_left", "exg"]

MAX_SENSOR_PAYLOAD_SIZE = 192
PARSE_RESYNC_SCAN_BYTES = 512


def _record_payload_size_is_valid(sensor_id: int, payload_size: int) -> bool:
    if payload_size <= 0 or payload_size > MAX_SENSOR_PAYLOAD_SIZE:
        return False

    if sensor_id not in SENSOR_FORMATS:
        return False

    expected_size = struct.calcsize(SENSOR_FORMATS[sensor_id])
    if payload_size == expected_size:
        return True

    if sensor_id in (SENSOR_SID["ppg_right"], SENSOR_SID["ppg_left"]):
        return payload_size > 2 and (payload_size - 2) % expected_size == 0

    return False


def _record_header_is_valid(sensor_id: int, payload_size: int) -> bool:
    return _record_payload_size_is_valid(sensor_id, payload_size)


def _find_resync_offset(f, record_start: int, file_size: int) -> Optional[int]:
    if record_start + SENSOR_PACKET_HEADER_SIZE >= file_size:
        return None

    max_window = min(
        PARSE_RESYNC_SCAN_BYTES + SENSOR_PACKET_HEADER_SIZE + MAX_SENSOR_PAYLOAD_SIZE + SENSOR_PACKET_HEADER_SIZE,
        file_size - record_start,
    )
    f.seek(record_start)
    window = bytearray(f.read(max_window))

    for shift in range(1, min(PARSE_RESYNC_SCAN_BYTES, len(window) - SENSOR_PACKET_HEADER_SIZE) + 1):
        if shift + SENSOR_PACKET_HEADER_SIZE > len(window):
            break

        sensor_id, payload_size, _ = struct.unpack_from(
            SENSOR_PACKET_HEADER_FORMAT, window, shift
        )
        if not _record_header_is_valid(sensor_id, payload_size):
            continue

        next_header_offset = shift + SENSOR_PACKET_HEADER_SIZE + payload_size
        if next_header_offset == len(window):
            return shift

        if next_header_offset + SENSOR_PACKET_HEADER_SIZE > len(window):
            continue

        next_sensor_id, next_payload_size, _ = struct.unpack_from(
            SENSOR_PACKET_HEADER_FORMAT, window, next_header_offset
        )
        if _record_header_is_valid(next_sensor_id, next_payload_size):
            return shift

    return None


def parse_oe_file(filename: str) -> tuple[pd.DataFrame, dict]:
    """Parse .oe file and return DataFrame and metadata."""
    FILE_HEADER_FORMAT = '<HQ'
    FILE_HEADER_SIZE = struct.calcsize(FILE_HEADER_FORMAT)
    FILE_HEADER_V3_FORMAT = '<HQIIQB'
    FILE_HEADER_V3_SIZE = struct.calcsize(FILE_HEADER_V3_FORMAT)
    
    data = defaultdict(list)
    metadata = {}
    _sid = None

    file_size = os.path.getsize(filename)
    if file_size == 0:
        raise ValueError(
            "File is empty (0 bytes). Recording did not contain data or was not finalized."
        )

    with open(filename, 'rb') as f:
        file_header = f.read(FILE_HEADER_SIZE)
        if len(file_header) < FILE_HEADER_SIZE:
            raise ValueError(
                f"File header is truncated (expected {FILE_HEADER_SIZE} bytes, got {len(file_header)})."
            )

        version, timestamp = struct.unpack(FILE_HEADER_FORMAT, file_header)
        metadata['version'] = version
        metadata['timestamp'] = timestamp
        metadata['filename'] = os.path.basename(filename)
        metadata['filesize_bytes'] = file_size
        metadata['truncated'] = False
        metadata['header_size'] = FILE_HEADER_SIZE
        metadata['resync_count'] = 0
        metadata['resync_positions'] = []

        if version == 0x0003:
            f.seek(0)
            file_header_v3 = f.read(FILE_HEADER_V3_SIZE)
            if len(file_header_v3) < FILE_HEADER_V3_SIZE:
                raise ValueError(
                    f"Version 0x0003 header is truncated (expected {FILE_HEADER_V3_SIZE} bytes, got {len(file_header_v3)})."
                )

            version, timestamp, header_size, parse_info_size, device_id, side = struct.unpack(
                FILE_HEADER_V3_FORMAT, file_header_v3
            )

            if header_size < FILE_HEADER_V3_SIZE or header_size > file_size:
                raise ValueError(
                    f"Invalid v0x0003 header size {header_size} for file size {file_size}."
                )

            metadata['timestamp'] = timestamp
            metadata['header_size'] = header_size
            metadata['parse_info_size'] = parse_info_size
            metadata['device_id'] = device_id
            metadata['side'] = side
            f.seek(header_size)

        while True:
            record_start = f.tell()
            header = f.read(10)
            if len(header) == 0:
                break
            if len(header) < 10:
                metadata['truncated'] = True
                break
            sid, size, time_us = struct.unpack('<BBQ', header)
            if not _record_header_is_valid(sid, size):
                resync_offset = _find_resync_offset(f, record_start, file_size)
                if resync_offset is None:
                    if _sid is not None and _sid in data.keys():
                        data[_sid].pop()
                    metadata['truncated'] = True
                    break

                metadata['resync_count'] += 1
                metadata['resync_positions'].append(record_start)
                f.seek(record_start + resync_offset)
                continue

            _sid = sid
            raw_data = f.read(size)
            if len(raw_data) < size:
                metadata['truncated'] = True
                break
            timestamp_s = time_us / 1e6

            try:
                if sid == SENSOR_SID["microphone"] or sid == SENSOR_SID["bone_acc"]:
                    continue
                elif sid in SENSOR_FORMATS:
                    fmt = SENSOR_FORMATS[sid]
                    expected_size = struct.calcsize(fmt)

                    if size == expected_size:
                        values = struct.unpack(fmt, raw_data)
                        data[sid].append((timestamp_s, values))
                    elif (size - 2) % expected_size == 0:
                        delta = struct.unpack('<H', raw_data[-2:])[0] / 1e6
                        raw_data = raw_data[:-2]
                        for n in range(len(raw_data) // expected_size):
                            values = struct.unpack(fmt, raw_data[n * expected_size: (n + 1) * expected_size])
                            data[sid].append((timestamp_s + n * delta, values))
            except struct.error:
                pass

    # Build DataFrame
    dfs = []
    sensor_counts = {}
    
    for name in ACTIVE_SENSORS:
        sid = SENSOR_SID[name]
        labels = LABELS.get(name, [])
        sensor_counts[name] = len(data.get(sid, []))
        
        if sid in data and data[sid]:
            times, values = zip(*data[sid])
            df = pd.DataFrame(values, columns=labels)
            df['timestamp'] = times
            df.set_index('timestamp', inplace=True)
            df = df[~df.index.duplicated(keep='first')]
            dfs.append(df)

    metadata['sensor_counts'] = sensor_counts

    if dfs:
        common_index = pd.Index([])
        for df in dfs:
            common_index = common_index.union(df.index)
        common_index = common_index.sort_values()
        reindexed_dfs = [df.reindex(common_index) for df in dfs]
        result_df = pd.concat(reindexed_dfs, axis=1)
    else:
        result_df = pd.DataFrame()

    return result_df, metadata


# =============================================================================
# GUI
# =============================================================================

class WorkerSignals(QObject):
    scan_complete = pyqtSignal(object)  # (list[BLEDevice], int) tuple
    scan_progress = pyqtSignal(object)  # (list[BLEDevice], int, elapsed_s, total_s)
    scan_error = pyqtSignal(str)
    sync_complete = pyqtSignal(bool, str)
    progress = pyqtSignal(str)
    parse_complete = pyqtSignal(object, dict)
    parse_error = pyqtSignal(str)


class TimeSyncTab(QWidget):
    def __init__(self, signals: WorkerSignals):
        super().__init__()
        self.signals = signals
        self.devices: list[BLEDevice] = []
        self.before_sync_callback: Optional[Callable[[], None]] = None
        self.ble_loop = asyncio.new_event_loop()
        self.scan_stop_event = threading.Event()
        self.scan_in_progress = False
        self.sync_in_progress = False
        self.scan_thread: Optional[threading.Thread] = None
        self._create_widgets()

    def _create_widgets(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Device list group
        group = QGroupBox("Found Devices")
        group_layout = QVBoxLayout(group)

        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Name", "RSSI (dBm)", "Address"])
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        group_layout.addWidget(self.table)

        layout.addWidget(group)

        # Status label
        self.status_label = QLabel("Click Refresh to scan for devices...")
        self.status_label.setStyleSheet("color: gray;")
        layout.addWidget(self.status_label)

        # Recording schedule settings
        schedule_group = QGroupBox("Scheduled Recording")
        schedule_layout = QFormLayout(schedule_group)

        self.participant_id_input = QLineEdit()
        self.participant_id_input.setPlaceholderText(f"e.g. {DEFAULT_PARTICIPANT_ID}")
        self.participant_id_input.setText(DEFAULT_PARTICIPANT_ID)
        schedule_layout.addRow("Participant ID:", self.participant_id_input)

        self.start_time_picker = QDateTimeEdit()
        self.start_time_picker.setCalendarPopup(True)
        self.start_time_picker.setDisplayFormat("dd. MMM yyyy, HH:mm:ss")
        self.start_time_picker.setDateTime(QDateTime.currentDateTime().addSecs(60))
        schedule_layout.addRow("Start date/time:", self.start_time_picker)

        layout.addWidget(schedule_group)

        # Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.start_scan)
        btn_layout.addWidget(self.refresh_btn)

        self.sync_btn = QPushButton("⏸️ Sync + Schedule Recording")
        self.sync_btn.clicked.connect(self.start_sync)
        btn_layout.addWidget(self.sync_btn)

        self.run_now_btn = QPushButton("▶️ Sync + Run Immediately")
        self.run_now_btn.clicked.connect(self.start_sync_now)
        btn_layout.addWidget(self.run_now_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        # Progress bar
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setVisible(False)
        layout.addWidget(self.progress)

    def set_buttons_enabled(self, enabled: bool):
        self.refresh_btn.setEnabled(enabled)
        self.sync_btn.setEnabled(enabled)
        self.run_now_btn.setEnabled(enabled)
        self.progress.setVisible(not enabled)

    def is_busy(self) -> bool:
        return self.sync_in_progress

    def _set_idle_controls(self):
        self.refresh_btn.setEnabled(True)
        has_devices = len(self.devices) > 0
        self.sync_btn.setEnabled(has_devices)
        self.run_now_btn.setEnabled(has_devices)
        self.progress.setVisible(False)

    def start_scan(self):
        if self.scan_in_progress or self.sync_in_progress:
            return
        self.scan_stop_event.clear()
        self.scan_in_progress = True
        self.set_buttons_enabled(False)
        self.status_label.setText("Scanning for devices...")
        self.table.setRowCount(0)
        self.scan_thread = threading.Thread(target=self._scan_thread, daemon=True)
        self.scan_thread.start()

    def _scan_thread(self):
        def progress_cb(devices, total_scanned, elapsed_s, total_s):
            self.signals.scan_progress.emit((devices, total_scanned, elapsed_s, total_s))

        try:
            devices, total_scanned = self.ble_loop.run_until_complete(
                scan_devices(
                    timeout=BLE_SCAN_TIMEOUT_SECONDS,
                    progress_callback=progress_cb,
                    should_stop=self.scan_stop_event.is_set,
                )
            )
            self.signals.scan_complete.emit((devices, total_scanned))
        except Exception as e:
            self.signals.scan_error.emit(str(e))

    def _stop_scan_if_running(self) -> bool:
        if self.scan_in_progress:
            self.scan_stop_event.set()
            if self.scan_thread and self.scan_thread.is_alive():
                self.scan_thread.join(timeout=BLE_SCAN_CHUNK_SECONDS + 4.0)
                if self.scan_thread.is_alive():
                    return False
        return True

    def _render_scan_results(self, devices: list[BLEDevice]):
        self.table.setRowCount(len(devices))
        for i, device in enumerate(devices):
            self.table.setItem(i, 0, QTableWidgetItem(device.name or "Unknown"))
            rssi = get_ble_rssi(device)
            self.table.setItem(i, 1, QTableWidgetItem(str(rssi if rssi is not None else "N/A")))
            self.table.setItem(i, 2, QTableWidgetItem(device.address))
        if devices and self.table.currentRow() < 0:
            self.table.selectRow(0)

    def on_scan_progress(self, result):
        devices, total_scanned, elapsed_s, total_s = result
        self.devices = devices
        self._render_scan_results(devices)
        if devices and not self.sync_in_progress:
            self.sync_btn.setEnabled(True)
            self.run_now_btn.setEnabled(True)
        self.status_label.setText(
            f"Scanning... found {len(devices)} OpenEarable device(s) "
            f"({int(elapsed_s)}/{int(total_s)} s, {total_scanned} BLE devices seen)"
        )

    def on_scan_complete(self, result):
        devices, total_scanned = result
        self.devices = devices
        self.scan_in_progress = False
        self.scan_thread = None
        if not self.sync_in_progress:
            self._set_idle_controls()
        self._render_scan_results(devices)
        if self.sync_in_progress:
            return

        if devices:
            self.status_label.setText(f"Found {len(devices)} OpenEarable device(s). Select one and click Sync.")
        else:
            self.status_label.setText(f"No OpenEarable found (scanned {total_scanned} BLE devices). Make sure device is on and advertising.")

    def on_scan_error(self, error: str):
        self.scan_in_progress = False
        self.scan_thread = None
        if not self.sync_in_progress:
            self._set_idle_controls()
            self.status_label.setText(f"Scan error: {error}")
            QMessageBox.critical(self, "Scan Error", error)

    def start_sync(self):
        selected_qdt = self.start_time_picker.dateTime()
        scheduled_start_us = selected_qdt.toSecsSinceEpoch() * 1_000_000
        now_us = int(time.time() * 1_000_000)
        if scheduled_start_us <= now_us:
            QMessageBox.warning(
                self,
                "Invalid Start Time",
                "Start date/time must be in the future.",
            )
            return
        self._start_sync_internal(scheduled_start_us)

    def start_sync_now(self):
        # Give BLE enough setup time before recording starts.
        scheduled_start_us = int(time.time() * 1_000_000) + RUN_NOW_LEAD_TIME_US
        self.start_time_picker.setDateTime(QDateTime.fromSecsSinceEpoch(scheduled_start_us // 1_000_000))
        self._start_sync_internal(scheduled_start_us)

    def _start_sync_internal(self, scheduled_start_us: int):
        if self.sync_in_progress:
            return
        if self.before_sync_callback:
            try:
                self.before_sync_callback()
            except Exception:
                pass
        if not self._stop_scan_if_running():
            QMessageBox.warning(self, "BLE Busy", "BLE scan is still stopping. Please try again in a moment.")
            return

        selected = self.table.selectedItems()
        if not selected and self.devices:
            self.table.selectRow(0)
            selected = self.table.selectedItems()
        if not selected:
            QMessageBox.warning(self, "No Selection", "Please select a device first.")
            return

        row = self.table.currentRow()
        if row < 0 or row >= len(self.devices):
            return

        device = self.devices[row]
        device_name = device.name or device.address
        participant_id = self.participant_id_input.text().strip()
        if not participant_id:
            participant_id = DEFAULT_PARTICIPANT_ID
            self.participant_id_input.setText(participant_id)

        self.sync_in_progress = True
        self.set_buttons_enabled(False)
        self.status_label.setText(f"Connecting to {device_name}...")

        thread = threading.Thread(
            target=self._sync_thread,
            args=(device, participant_id, scheduled_start_us),
            daemon=True,
        )
        thread.start()

    def _sync_thread(self, device: BLEDevice, participant_id: str, scheduled_start_us: int):
        def progress_cb(msg):
            self.signals.progress.emit(msg)

        try:
            success, message = self.ble_loop.run_until_complete(
                sync_time_and_schedule_exg_recording(
                    device,
                    participant_id,
                    scheduled_start_us,
                    progress_callback=progress_cb,
                )
            )
            self.signals.sync_complete.emit(success, message)
        except Exception as e:
            self.signals.sync_complete.emit(False, str(e))

    def on_progress(self, msg: str):
        self.status_label.setText(msg)

    def on_sync_complete(self, success: bool, message: str):
        self.sync_in_progress = False
        if self.scan_in_progress:
            self.set_buttons_enabled(False)
        else:
            self._set_idle_controls()
        if success:
            self.status_label.setText("✓ Time sync + scheduled recording successful!")
            QMessageBox.information(self, "Success", message)
        else:
            self.status_label.setText("✗ Scheduled sync failed")
            QMessageBox.critical(self, "Sync Failed", message)


class USBTimeSyncTab(QWidget):
    """USB Time Sync tab with debug console."""
    
    debug_message = pyqtSignal(str)
    
    def __init__(self, signals: WorkerSignals):
        super().__init__()
        self.signals = signals
        self.ports: list[tuple[str, str, str]] = []
        self._create_widgets()
        
        # Connect debug signal to console
        self.debug_message.connect(self._append_debug)

    def _create_widgets(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Device list group
        group = QGroupBox("Found Devices")
        group_layout = QVBoxLayout(group)

        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Port", "Name", "Hardware ID"])
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        group_layout.addWidget(self.table)

        layout.addWidget(group)

        # Status label
        self.status_label = QLabel("Click Refresh to scan for USB ports...")
        self.status_label.setStyleSheet("color: gray;")
        layout.addWidget(self.status_label)

        # Recording schedule settings
        schedule_group = QGroupBox("Scheduled Recording")
        schedule_layout = QFormLayout(schedule_group)

        self.participant_id_input = QLineEdit()
        self.participant_id_input.setPlaceholderText(f"e.g. {DEFAULT_PARTICIPANT_ID}")
        self.participant_id_input.setText(DEFAULT_PARTICIPANT_ID)
        schedule_layout.addRow("Participant ID:", self.participant_id_input)

        self.start_time_picker = QDateTimeEdit()
        self.start_time_picker.setCalendarPopup(True)
        self.start_time_picker.setDisplayFormat("dd. MMM yyyy, HH:mm:ss")
        self.start_time_picker.setDateTime(QDateTime.currentDateTime().addSecs(60))
        schedule_layout.addRow("Start date/time:", self.start_time_picker)

        layout.addWidget(schedule_group)

        # Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        btn_layout.addWidget(self.refresh_btn)

        self.sync_btn = QPushButton("⏸️ Sync + Schedule Recording")
        self.sync_btn.clicked.connect(self.start_sync)
        btn_layout.addWidget(self.sync_btn)

        self.run_now_btn = QPushButton("▶️ Sync + Run Immediately")
        self.run_now_btn.clicked.connect(self.start_sync_now)
        btn_layout.addWidget(self.run_now_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        # Progress bar
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setVisible(False)
        layout.addWidget(self.progress)

        # Debug console (only shown if DEBUG is True)
        if DEBUG:
            debug_group = QGroupBox("🔧 Debug Console")
            debug_layout = QVBoxLayout(debug_group)
            
            debug_header = QHBoxLayout()
            
            self.auto_scroll_cb = QCheckBox("Auto-scroll")
            self.auto_scroll_cb.setChecked(True)
            debug_header.addWidget(self.auto_scroll_cb)
            
            self.clear_btn = QPushButton("Clear")
            self.clear_btn.clicked.connect(self._clear_debug)
            debug_header.addWidget(self.clear_btn)
            
            debug_header.addStretch()
            debug_layout.addLayout(debug_header)
            
            self.debug_console = QTextEdit()
            self.debug_console.setReadOnly(True)
            self.debug_console.setFont(QFont("Menlo" if sys.platform == "darwin" else "Consolas", 10))
            self.debug_console.setStyleSheet("""
                QTextEdit {
                    background-color: #1e1e1e;
                    color: #d4d4d4;
                    border: 1px solid #3c3c3c;
                }
            """)
            self.debug_console.setMaximumHeight(150)
            debug_layout.addWidget(self.debug_console)
            
            layout.addWidget(debug_group)

        # Info box at bottom
        if not SERIAL_AVAILABLE:
            info = QLabel("⚠️ pyserial not installed. Run: pip install pyserial")
            info.setStyleSheet("color: #ff6b6b; font-size: 12px; padding: 5px;")    
            info.setWordWrap(True)
            layout.addWidget(info)

    def _append_debug(self, msg: str):
        """Append message to debug console (thread-safe via signal)."""
        if not DEBUG or not hasattr(self, 'debug_console'):
            return
        self.debug_console.append(msg)
        if self.auto_scroll_cb.isChecked():
            cursor = self.debug_console.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self.debug_console.setTextCursor(cursor)

    def _clear_debug(self):
        """Clear the debug console."""
        if DEBUG and hasattr(self, 'debug_console'):
            self.debug_console.clear()

    def refresh_ports(self):
        """Scan for USB serial ports."""
        self._append_debug(f"\n--- Scanning USB ports at {time.strftime('%H:%M:%S')} ---")
        
        if not SERIAL_AVAILABLE:
            self._append_debug("ERROR: pyserial not installed!")
            self.status_label.setText("⚠️ pyserial not installed")
            return
        
        self.ports = find_openearable_usb_ports()
        self.table.setRowCount(len(self.ports))
        
        for i, (port, desc, hwid) in enumerate(self.ports):
            self.table.setItem(i, 0, QTableWidgetItem(port))
            self.table.setItem(i, 1, QTableWidgetItem(desc))
            self.table.setItem(i, 2, QTableWidgetItem(hwid))
            self._append_debug(f"  Found: {port} - {desc}")
        
        if self.ports:
            self.status_label.setText(f"Found {len(self.ports)} port(s). Select one and click Sync.")
            self.table.selectRow(0)
            self._append_debug(f"Found {len(self.ports)} candidate port(s)")
        else:
            self.status_label.setText("No USB serial ports found. Connect device and click Refresh.")
            self._append_debug("No candidate ports found. Is the device connected?")
            
            # Show all ports for debugging
            all_ports = list(serial.tools.list_ports.comports())
            if all_ports:
                self._append_debug(f"\nAll system ports ({len(all_ports)}):")
                for p in all_ports:
                    self._append_debug(f"  {p.device}: {p.description} (VID={p.vid}, PID={p.pid})")

    def set_buttons_enabled(self, enabled: bool):
        self.refresh_btn.setEnabled(enabled)
        self.sync_btn.setEnabled(enabled and SERIAL_AVAILABLE)
        self.run_now_btn.setEnabled(enabled and SERIAL_AVAILABLE)
        self.progress.setVisible(not enabled)

    def is_busy(self) -> bool:
        return not self.sync_btn.isEnabled()

    def start_sync(self):
        selected_qdt = self.start_time_picker.dateTime()
        scheduled_start_us = selected_qdt.toSecsSinceEpoch() * 1_000_000
        now_us = int(time.time() * 1_000_000)
        if scheduled_start_us <= now_us:
            QMessageBox.warning(
                self,
                "Invalid Start Time",
                "Start date/time must be in the future.",
            )
            return
        self._start_sync_internal(scheduled_start_us)

    def start_sync_now(self):
        # Give USB enough setup time before recording starts.
        scheduled_start_us = int(time.time() * 1_000_000) + RUN_NOW_LEAD_TIME_US
        self.start_time_picker.setDateTime(QDateTime.fromSecsSinceEpoch(scheduled_start_us // 1_000_000))
        self._start_sync_internal(scheduled_start_us)

    def _start_sync_internal(self, scheduled_start_us: int):
        selected = self.table.selectedItems()
        if not selected:
            QMessageBox.warning(self, "No Selection", "Please select a port first.")
            return

        row = self.table.currentRow()
        if row < 0 or row >= len(self.ports):
            return

        participant_id = self.participant_id_input.text().strip()
        if not participant_id:
            participant_id = DEFAULT_PARTICIPANT_ID
            self.participant_id_input.setText(participant_id)

        port = self.ports[row][0]
        self.set_buttons_enabled(False)
        self.status_label.setText(f"Syncing + scheduling via {port}...")

        self._append_debug(f"\n{'='*50}")
        self._append_debug(f"Starting USB sync + schedule on {port}")
        self._append_debug(f"{'='*50}")

        thread = threading.Thread(
            target=self._sync_thread,
            args=(port, participant_id, scheduled_start_us),
            daemon=True,
        )
        thread.start()

    def _sync_thread(self, port: str, participant_id: str, scheduled_start_us: int):
        """Background thread for USB sync + scheduling."""
        def progress_cb(msg):
            self.signals.progress.emit(msg)

        def debug_cb(msg):
            # Use signal for thread-safe GUI update
            self.debug_message.emit(msg)

        try:
            success, message = sync_time_and_schedule_exg_recording_usb(
                port,
                participant_id,
                scheduled_start_us,
                progress_callback=progress_cb,
                debug_callback=debug_cb,
            )
            self.signals.sync_complete.emit(success, message)
        except Exception as e:
            self.debug_message.emit(f"EXCEPTION: {e}")
            self.signals.sync_complete.emit(False, str(e))

    def on_progress(self, msg: str):
        self.status_label.setText(msg)

    def on_sync_complete(self, success: bool, message: str):
        self.set_buttons_enabled(True)
        if success:
            self.status_label.setText("✓ USB time sync + scheduled recording successful!")
            self._append_debug(f"\n✓ SUCCESS!")
            QMessageBox.information(self, "Success", message)
        else:
            self.status_label.setText("✗ USB sync + schedule failed")
            self._append_debug(f"\n✗ FAILED: {message}")
            QMessageBox.critical(self, "Sync Failed", message)


class UnifiedSyncTab(QWidget):
    def __init__(self, signals: WorkerSignals):
        super().__init__()
        self.ble_tab = TimeSyncTab(signals)
        self.usb_tab = USBTimeSyncTab(signals)
        self._create_widgets()

    def _create_widgets(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        mode_row = QHBoxLayout()
        mode_label = QLabel("Sync mode:")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Bluetooth (BLE)", "USB"])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_row.addWidget(mode_label)
        mode_row.addWidget(self.mode_combo)
        mode_row.addStretch()
        layout.addLayout(mode_row)

        self.stack = QStackedWidget()
        self.stack.addWidget(self.ble_tab)
        self.stack.addWidget(self.usb_tab)
        layout.addWidget(self.stack)

    def _on_mode_changed(self, index: int):
        self.stack.setCurrentIndex(index)

    def start_ble_scan(self):
        self.ble_tab.start_scan()

    def start_usb_scan(self):
        self.usb_tab.refresh_ports()


class ExGLivePlotTab(QWidget):
    scan_progress = pyqtSignal(object)
    scan_complete = pyqtSignal(object)
    scan_error = pyqtSignal(str)
    sample_received = pyqtSignal(float)
    status_update = pyqtSignal(str)
    stream_finished = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.devices: list[BLEDevice] = []
        self.loop = asyncio.new_event_loop()
        self.scan_stop_event = threading.Event()
        self.scan_in_progress = False
        self.scan_thread: Optional[threading.Thread] = None
        self.stream_stop_event = threading.Event()
        self.stream_running = False
        self.stream_thread: Optional[threading.Thread] = None
        self.is_closing = False
        self.display_max_samples = 500
        self.stream_sample_rate_hz = 256.0
        self.min_buffer_uv = 400.0
        self.autoscale = False
        self.enable_filters = True
        self.samples = deque(maxlen=self.display_max_samples)
        self.sample_lock = threading.Lock()
        self.plot_dirty = False
        self.figure = Figure(figsize=(7.5, 2.6), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.line, = self.ax.plot([], [])
        self._reset_filter_state()
        self._init_plot()
        self._create_widgets()
        self._connect_signals()

    def _reset_filter_state(self):
        self.filters = digitalfilter.get_Biopotential_filter(
            order=4,
            cutoff=[1, 30],
            btype="bandpass",
            fs=self.stream_sample_rate_hz,
            output="sos",
        )

    def _init_plot(self):
        self.ax.clear()
        self.line, = self.ax.plot([], [], color="#2c7fb8", linewidth=1.0)
        self.ax.set_xlim(0, self.display_max_samples)
        self.ax.set_title("Biopotential Data from OpenEarable ExG")
        self.ax.set_ylabel("Voltage (\u00b5V)")
        self.ax.set_xlabel("Samples")
        self.ax.grid(True, alpha=0.3)
        self.figure.subplots_adjust(left=0.08, right=0.99, bottom=0.20, top=0.88)
        if not self.autoscale:
            self.ax.set_ylim(-self.min_buffer_uv, self.min_buffer_uv)

    def _push_sample(self, value_uv: float):
        if self.is_closing:
            return
        filtered = float(self.filters(value_uv))
        with self.sample_lock:
            self.samples.append(filtered if self.enable_filters else value_uv)
            self.plot_dirty = True

    def _create_widgets(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        group = QGroupBox("Found Devices")
        group_layout = QVBoxLayout(group)

        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Name", "RSSI (dBm)", "Address"])
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        group_layout.addWidget(self.table)
        layout.addWidget(group)

        self.status_label = QLabel("Click Refresh to scan for devices...")
        self.status_label.setStyleSheet("color: gray;")
        layout.addWidget(self.status_label)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.start_scan)
        btn_layout.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("▶️ Connect + Stream ExG")
        self.connect_btn.clicked.connect(self.start_stream)
        btn_layout.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("⏹ Stop Stream")
        self.disconnect_btn.clicked.connect(self.stop_stream)
        self.disconnect_btn.setEnabled(False)
        btn_layout.addWidget(self.disconnect_btn)

        self.clear_btn = QPushButton("🧹 Clear Plot")
        self.clear_btn.clicked.connect(self.clear_plot)
        btn_layout.addWidget(self.clear_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setVisible(False)
        layout.addWidget(self.progress)

        plot_group = QGroupBox("Live ExG Plot")
        plot_layout = QVBoxLayout(plot_group)
        self.plot_label = QLabel("No data yet. Start stream to view filtered ExG (1-30 Hz).")
        self.plot_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.plot_label.setMinimumHeight(260)
        self.plot_label.setStyleSheet("background: #ffffff; border: 1px solid #d0d0d0;")
        plot_layout.addWidget(self.plot_label)
        layout.addWidget(plot_group)

        self.plot_timer = QTimer(self)
        self.plot_timer.setInterval(20)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start()

    def _connect_signals(self):
        self.scan_progress.connect(self.on_scan_progress)
        self.scan_complete.connect(self.on_scan_complete)
        self.scan_error.connect(self.on_scan_error)
        self.sample_received.connect(self.on_sample_received)
        self.status_update.connect(self.on_status_update)
        self.stream_finished.connect(self.on_stream_finished)

    def set_buttons_enabled(self, enabled: bool):
        self.refresh_btn.setEnabled(enabled and not self.stream_running)
        self.connect_btn.setEnabled(enabled and not self.stream_running)
        self.disconnect_btn.setEnabled(self.stream_running)
        self.progress.setVisible(not enabled and not self.stream_running)

    def start_scan(self):
        if self.stream_running or self.is_closing:
            return
        self.scan_stop_event.clear()
        self.scan_in_progress = True
        self.set_buttons_enabled(False)
        self.status_label.setText("Scanning for devices...")
        self.table.setRowCount(0)
        self.scan_thread = threading.Thread(target=self._scan_thread, daemon=True)
        self.scan_thread.start()

    def _scan_thread(self):
        def progress_cb(devices, total_scanned, elapsed_s, total_s):
            self.scan_progress.emit((devices, total_scanned, elapsed_s, total_s))

        try:
            devices, total_scanned = self.loop.run_until_complete(
                scan_devices(
                    timeout=BLE_SCAN_TIMEOUT_SECONDS,
                    progress_callback=progress_cb,
                    should_stop=self.scan_stop_event.is_set,
                )
            )
            self.scan_complete.emit((devices, total_scanned))
        except Exception as e:
            self.scan_error.emit(str(e))

    def _stop_scan_if_running(self):
        if self.scan_in_progress:
            self.scan_stop_event.set()
            if self.scan_thread and self.scan_thread.is_alive():
                self.scan_thread.join(timeout=BLE_SCAN_CHUNK_SECONDS + 1.5)

    def stop_scan(self):
        self._stop_scan_if_running()

    def _render_scan_results(self, devices: list[BLEDevice]):
        self.table.setRowCount(len(devices))
        for i, device in enumerate(devices):
            self.table.setItem(i, 0, QTableWidgetItem(device.name or "Unknown"))
            rssi = get_ble_rssi(device)
            self.table.setItem(i, 1, QTableWidgetItem(str(rssi if rssi is not None else "N/A")))
            self.table.setItem(i, 2, QTableWidgetItem(device.address))
        if devices and self.table.currentRow() < 0:
            self.table.selectRow(0)

    def on_scan_progress(self, result):
        devices, total_scanned, elapsed_s, total_s = result
        self.devices = devices
        self._render_scan_results(devices)
        if devices and not self.stream_running:
            self.connect_btn.setEnabled(True)
        self.status_label.setText(
            f"Scanning... found {len(devices)} OpenEarable device(s) "
            f"({int(elapsed_s)}/{int(total_s)} s, {total_scanned} BLE devices seen)"
        )

    def on_scan_complete(self, result):
        devices, total_scanned = result
        self.devices = devices
        self.scan_in_progress = False
        self.scan_thread = None
        self.set_buttons_enabled(True)
        self._render_scan_results(devices)

        if devices:
            self.status_label.setText(f"Found {len(devices)} OpenEarable device(s).")
        else:
            self.status_label.setText(
                f"No OpenEarable found (scanned {total_scanned} BLE devices)."
            )

    def on_scan_error(self, error: str):
        self.scan_in_progress = False
        self.scan_thread = None
        self.set_buttons_enabled(True)
        self.status_label.setText(f"Scan error: {error}")
        if not self.is_closing:
            QMessageBox.critical(self, "Scan Error", error)

    def start_stream(self):
        if self.stream_running or self.is_closing:
            return
        self._stop_scan_if_running()

        selected = self.table.selectedItems()
        if not selected:
            QMessageBox.warning(self, "No Selection", "Please select a device first.")
            return

        row = self.table.currentRow()
        if row < 0 or row >= len(self.devices):
            return

        device = self.devices[row]
        self.stream_running = True
        self.stream_stop_event.clear()
        with self.sample_lock:
            self.samples.clear()
            self.plot_dirty = True
            self._reset_filter_state()
        self.set_buttons_enabled(True)
        self.status_label.setText(f"Connecting to {device.name}...")

        self.stream_thread = threading.Thread(target=self._stream_thread, args=(device,), daemon=True)
        self.stream_thread.start()

    def _stream_thread(self, device: BLEDevice):
        try:
            self.loop.run_until_complete(self._stream_device(device))
        finally:
            if not self.is_closing:
                self.stream_finished.emit()

    async def _stream_device(self, device: BLEDevice):
        if self.is_closing:
            return
        self.status_update.emit("Connecting...")
        try:
            async with BleakClient(device) as client:
                if self.is_closing:
                    return
                self.status_update.emit("Connected. Enabling ExG streaming...")
                # Keep ExG live stream behavior aligned with record_and_realtime_plot_BLE.py.
                stream_cfg = build_sensor_config(
                    SENSOR_ID_EXG, EXG_SAMPLERATE_INDEX_256HZ, DATA_STREAMING
                )
                await client.write_gatt_char(SENSOR_CONFIG_CHAR_UUID, stream_cfg)

                def on_notification(sender, data: bytearray):
                    if self.is_closing or self.stream_stop_event.is_set():
                        return
                    pkt = parse_sensor_packet(bytes(data))
                    if pkt is None:
                        return
                    sensor_id, payload_size, _, payload = pkt
                    if sensor_id != SENSOR_ID_EXG or payload_size < 4:
                        return
                    value_uv = struct.unpack("<f", payload[:4])[0]
                    self._push_sample(value_uv)

                await client.start_notify(SENSOR_DATA_CHAR_UUID, on_notification)
                self.status_update.emit("Streaming ExG data...")

                while not self.stream_stop_event.is_set():
                    await asyncio.sleep(0.05)

                await client.stop_notify(SENSOR_DATA_CHAR_UUID)

                stop_cfg = build_sensor_config(SENSOR_ID_EXG, 0, 0)
                await client.write_gatt_char(SENSOR_CONFIG_CHAR_UUID, stop_cfg)
                self.status_update.emit("ExG stream stopped.")
        except Exception as e:
            if self.is_closing:
                return
            self.status_update.emit(f"Stream error: {e}")

    def stop_stream(self):
        if self.stream_running:
            self.stream_stop_event.set()
            self.status_label.setText("Stopping stream...")

    def clear_plot(self):
        with self.sample_lock:
            self.samples.clear()
            self.plot_dirty = True
            self._reset_filter_state()
        self._init_plot()
        self.plot_label.setText("Plot cleared.")

    def on_sample_received(self, value_uv: float):
        self._push_sample(value_uv)

    def on_status_update(self, message: str):
        self.status_label.setText(message)

    def on_stream_finished(self):
        self.stream_running = False
        self.stream_thread = None
        self.disconnect_btn.setEnabled(False)
        self.refresh_btn.setEnabled(True)
        self.connect_btn.setEnabled(True)

    def _update_plot(self):
        if self.is_closing:
            return
        with self.sample_lock:
            if not self.plot_dirty:
                return
            values = list(self.samples)
            self.plot_dirty = False

        if len(values) < 2:
            return

        self.line.set_data(range(len(values)), values)
        min_val = min(values)
        max_val = max(values)
        buffer = (
            0.1 * (max_val - min_val)
            if (max_val - min_val) > self.min_buffer_uv
            else self.min_buffer_uv
        )
        if self.autoscale:
            self.ax.set_ylim(min_val - buffer, max_val + buffer)
        else:
            self.ax.set_ylim(-self.min_buffer_uv, self.min_buffer_uv)

        buf = BytesIO()
        self.figure.savefig(buf, format="png", facecolor="white")
        buf.seek(0)

        pixmap = QPixmap()
        pixmap.loadFromData(QByteArray(buf.getvalue()))
        self.plot_label.setPixmap(pixmap)
        self.plot_dirty = False

    def shutdown(self, timeout_s: float = 3.0):
        self.is_closing = True
        self.plot_timer.stop()
        self.stop_scan()
        self.stop_stream()
        if self.stream_thread and self.stream_thread.is_alive():
            self.stream_thread.join(timeout=timeout_s)
        if not self.loop.is_closed():
            self.loop.close()


class FileConverterTab(QWidget):
    def __init__(self, signals: WorkerSignals):
        super().__init__()
        self.signals = signals
        self.loaded_files: list[tuple[str, pd.DataFrame, dict]] = []
        self._create_widgets()

    def _create_widgets(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Create splitter for file list (top) and preview (bottom)
        splitter = QSplitter(Qt.Orientation.Vertical)

        # Top part: File list and buttons
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)

        # File list group (compact)
        group = QGroupBox("Loaded .oe Files")
        group_layout = QVBoxLayout(group)

        self.file_list = QListWidget()
        self.file_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        self.file_list.itemSelectionChanged.connect(self._on_selection_changed)
        self.file_list.setMaximumHeight(100)
        group_layout.addWidget(self.file_list)
        top_layout.addWidget(group)

        # Status label
        self.status_label = QLabel("Import .oe files to convert to CSV")
        self.status_label.setStyleSheet("color: gray;")
        top_layout.addWidget(self.status_label)

        # Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.import_btn = QPushButton("📂 Import .oe Files")
        self.import_btn.clicked.connect(self.import_files)
        btn_layout.addWidget(self.import_btn)

        self.export_btn = QPushButton("💾 Export to CSV")
        self.export_btn.clicked.connect(self.export_csv)
        self.export_btn.setEnabled(False)
        btn_layout.addWidget(self.export_btn)

        self.clear_btn = QPushButton("🗑 Clear")
        self.clear_btn.clicked.connect(self.clear_files)
        self.clear_btn.setEnabled(False)
        btn_layout.addWidget(self.clear_btn)

        btn_layout.addStretch()
        top_layout.addLayout(btn_layout)

        # Progress bar
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setVisible(False)
        top_layout.addWidget(self.progress)

        splitter.addWidget(top_widget)

        # Bottom part: File preview with plots (in scroll area)
        self.preview_group = QGroupBox("File Preview")
        preview_layout = QVBoxLayout(self.preview_group)
        
        # Scroll area for plots
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        
        self.preview_content = QWidget()
        self.preview_layout = QVBoxLayout(self.preview_content)
        self.preview_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        
        # Placeholder label
        self.preview_placeholder = QLabel("Select a file to see preview")
        self.preview_placeholder.setStyleSheet("color: gray; padding: 20px;")
        self.preview_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_layout.addWidget(self.preview_placeholder)
        
        scroll.setWidget(self.preview_content)
        preview_layout.addWidget(scroll)
        
        splitter.addWidget(self.preview_group)
        
        # Set splitter sizes (30% top, 70% bottom for preview)
        splitter.setSizes([150, 350])
        
        layout.addWidget(splitter)

    def _clear_preview(self):
        """Clear all widgets from preview layout."""
        while self.preview_layout.count():
            item = self.preview_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    def _create_sensor_plot(self, df: pd.DataFrame, sensor_name: str, labels: list, sample_count: int) -> QLabel:
        """Create a small plot for a sensor and return as QLabel with embedded image."""
        fig, ax = plt.subplots(figsize=(5, 1.5), dpi=100)
        
        colors = {
            'acc': ['#e74c3c', '#27ae60', '#3498db'],
            'gyro': ['#e74c3c', '#27ae60', '#3498db'],
            'mag': ['#e74c3c', '#27ae60', '#3498db'],
            'ppg_right': ['red', 'darkred', 'green', 'gray'],
            'ppg_left': ['salmon', 'maroon', 'lightgreen', 'lightgray'],
            'optical_temp_right': ['#e74c3c'],
            'optical_temp_left': ['#3498db'],
            'exg': ['#9b59b6'],
        }
        
        sensor_colors = colors.get(sensor_name, ['#3498db'] * len(labels))
        
        for i, label in enumerate(labels):
            if label in df.columns:
                series = df[label].dropna()
                if len(series) > 4:
                    series = series.iloc[2:-2]  # Trim edges
                if len(series) > 0:
                    color = sensor_colors[i % len(sensor_colors)]
                    short_label = label.split('.')[-1] if '.' in label else label
                    ax.plot(series.index, series.values, label=short_label, color=color, linewidth=0.5)
        
        title = sensor_name.replace('_', ' ').title()
        ax.set_title(f"{title} ({sample_count:,} samples)", fontsize=10, fontweight='bold')
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.tick_params(axis='both', labelsize=7)
        ax.grid(True, alpha=0.3)
        if ax.get_legend_handles_labels()[1]:
            ax.legend(fontsize=7, loc='upper right', ncol=min(len(labels), 4))
        
        plt.tight_layout()
        
        # Convert to QPixmap
        buf = BytesIO()
        fig.savefig(buf, format='png', bbox_inches='tight', facecolor='white')
        buf.seek(0)
        plt.close(fig)
        
        pixmap = QPixmap()
        pixmap.loadFromData(QByteArray(buf.getvalue()))
        
        label_widget = QLabel()
        label_widget.setPixmap(pixmap)
        label_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        return label_widget

    def _on_selection_changed(self):
        selected = self.file_list.selectedItems()
        self._clear_preview()
        
        if len(selected) == 1:
            idx = self.file_list.row(selected[0])
            if idx < len(self.loaded_files):
                _, df, metadata = self.loaded_files[idx]
                
                # File info header
                info_label = QLabel()
                info_text = f"<b>📄 {metadata['filename']}</b><br>"
                info_text += f"Version: {metadata['version']} | Rows: {len(df)}<br>"
                if metadata.get('resync_count', 0) > 0:
                    info_text += (
                        f"Recovered corrupt chunks: {metadata['resync_count']}"
                        f"{' | ' if metadata.get('truncated') else '<br>'}"
                    )
                if metadata.get('truncated'):
                    info_text += "File ended with truncated data<br>"
                sensors_with_data = [s for s, c in metadata['sensor_counts'].items() if c > 0]
                info_text += f"Sensors: {', '.join(sensors_with_data)}"
                info_label.setText(info_text)
                info_label.setStyleSheet("padding: 5px; background-color: #f0f0f0; border-radius: 5px;")
                self.preview_layout.addWidget(info_label)
                
                # Collect plots for sensors with data
                plot_widgets = []
                for sensor_name in ACTIVE_SENSORS:
                    count = metadata['sensor_counts'].get(sensor_name, 0)
                    if count > 0:
                        labels = LABELS.get(sensor_name, [])
                        # Filter to columns that exist in df
                        existing_labels = [l for l in labels if l in df.columns]
                        if existing_labels:
                            plot_widget = self._create_sensor_plot(df, sensor_name, existing_labels, count)
                            plot_widgets.append(plot_widget)
                
                # Stack plots vertically (one per row)
                for plot_widget in plot_widgets:
                    self.preview_layout.addWidget(plot_widget)
                
                # Add stretch at end
                self.preview_layout.addStretch()
                
        elif len(selected) > 1:
            label = QLabel(f"<b>{len(selected)} files selected</b><br>Select a single file to see preview.")
            label.setStyleSheet("color: gray; padding: 20px;")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.preview_layout.addWidget(label)
        else:
            self.preview_placeholder = QLabel("Select a file to see preview")
            self.preview_placeholder.setStyleSheet("color: gray; padding: 20px;")
            self.preview_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.preview_layout.addWidget(self.preview_placeholder)

    def import_files(self):
        files, _ = QFileDialog.getOpenFileNames(
            self,
            "Select .oe Files",
            "",
            "OpenEarable Files (*.oe);;All Files (*)"
        )
        if not files:
            return

        self.progress.setVisible(True)
        self.import_btn.setEnabled(False)
        self.status_label.setText(f"Parsing {len(files)} file(s)...")

        thread = threading.Thread(target=self._parse_thread, args=(files,), daemon=True)
        thread.start()

    def _parse_thread(self, files: list[str]):
        try:
            results = []
            for filepath in files:
                df, metadata = parse_oe_file(filepath)
                results.append((filepath, df, metadata))
            self.signals.parse_complete.emit(results, {})
        except Exception as e:
            self.signals.parse_error.emit(str(e))

    def on_parse_complete(self, results: list, _):
        self.progress.setVisible(False)
        self.import_btn.setEnabled(True)

        for filepath, df, metadata in results:
            self.loaded_files.append((filepath, df, metadata))
            self.file_list.addItem(f"📄 {metadata['filename']} ({len(df)} rows)")

        self.export_btn.setEnabled(len(self.loaded_files) > 0)
        self.clear_btn.setEnabled(len(self.loaded_files) > 0)
        self.status_label.setText(f"✓ Loaded {len(results)} file(s). Total: {len(self.loaded_files)}")

        if self.file_list.count() > 0:
            self.file_list.setCurrentRow(self.file_list.count() - 1)

    def on_parse_error(self, error: str):
        self.progress.setVisible(False)
        self.import_btn.setEnabled(True)
        self.status_label.setText(f"✗ Parse error: {error}")
        QMessageBox.critical(self, "Parse Error", error)

    def export_csv(self):
        selected = self.file_list.selectedItems()
        if not selected:
            QMessageBox.warning(self, "No Selection", "Please select file(s) to export.")
            return

        indices = [self.file_list.row(item) for item in selected]
        
        if len(indices) == 1:
            # Single file - ask for save location
            idx = indices[0]
            filepath, df, metadata = self.loaded_files[idx]
            default_name = os.path.splitext(filepath)[0] + ".csv"
            
            save_path, _ = QFileDialog.getSaveFileName(
                self,
                "Save CSV",
                default_name,
                "CSV Files (*.csv);;All Files (*)"
            )
            if save_path:
                df.to_csv(save_path, index_label='timestamp')
                self.status_label.setText(f"✓ Exported to {os.path.basename(save_path)}")
                QMessageBox.information(self, "Success", f"Exported to:\n{save_path}")
        else:
            # Multiple files - ask for directory
            directory = QFileDialog.getExistingDirectory(self, "Select Export Directory")
            if directory:
                exported = []
                for idx in indices:
                    filepath, df, metadata = self.loaded_files[idx]
                    csv_name = os.path.splitext(metadata['filename'])[0] + ".csv"
                    save_path = os.path.join(directory, csv_name)
                    df.to_csv(save_path, index_label='timestamp')
                    exported.append(csv_name)
                
                self.status_label.setText(f"✓ Exported {len(exported)} file(s)")
                QMessageBox.information(self, "Success", f"Exported {len(exported)} files to:\n{directory}")

    def clear_files(self):
        self.loaded_files.clear()
        self.file_list.clear()
        self._clear_preview()
        # Add placeholder back
        self.preview_placeholder = QLabel("Select a file to see preview")
        self.preview_placeholder.setStyleSheet("color: gray; padding: 20px;")
        self.preview_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_layout.addWidget(self.preview_placeholder)
        self.export_btn.setEnabled(False)
        self.clear_btn.setEnabled(False)
        self.status_label.setText("Import .oe files to convert to CSV")


class OpenEarableToolApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenEarable Tools")
        self.setMinimumSize(550, 500)
        self.resize(600, 700)

        self.signals = WorkerSignals()
        self._create_widgets()
        self._connect_signals()
        self.sync_tab.ble_tab.before_sync_callback = self._prepare_ble_for_sync

        # Auto-scan on startup
        QTimer.singleShot(100, self.sync_tab.start_ble_scan)
        QTimer.singleShot(200, self.sync_tab.start_usb_scan)

    def _create_widgets(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 10, 10, 10)

        # Title
        title = QLabel("OpenEarable Tools")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Tab widget
        self.tabs = QTabWidget()
        
        self.sync_tab = UnifiedSyncTab(self.signals)
        self.tabs.addTab(self.sync_tab, "⏱️ Sync Time")

        self.exg_live_tab = ExGLivePlotTab()
        self.tabs.addTab(self.exg_live_tab, "📈 ExG Live")

        self.file_tab = FileConverterTab(self.signals)
        self.tabs.addTab(self.file_tab, "📂 File Converter")

        layout.addWidget(self.tabs)

    def _connect_signals(self):
        # BLE Time sync signals
        self.signals.scan_progress.connect(self.sync_tab.ble_tab.on_scan_progress)
        self.signals.scan_complete.connect(self.sync_tab.ble_tab.on_scan_complete)
        self.signals.scan_error.connect(self.sync_tab.ble_tab.on_scan_error)
        self.signals.sync_complete.connect(self._on_sync_complete)
        self.signals.progress.connect(self._on_progress)

        # File converter signals
        self.signals.parse_complete.connect(self.file_tab.on_parse_complete)
        self.signals.parse_error.connect(self.file_tab.on_parse_error)

    def _on_sync_complete(self, success: bool, message: str):
        """Route sync complete signal to tabs that may be running sync work."""
        if self.sync_tab.ble_tab.is_busy():
            self.sync_tab.ble_tab.on_sync_complete(success, message)
        if self.sync_tab.usb_tab.is_busy():
            self.sync_tab.usb_tab.on_sync_complete(success, message)

    def _on_progress(self, msg: str):
        """Route progress updates to tabs that currently run sync work."""
        if self.sync_tab.ble_tab.is_busy():
            self.sync_tab.ble_tab.on_progress(msg)
        if self.sync_tab.usb_tab.is_busy():
            self.sync_tab.usb_tab.on_progress(msg)

    def _prepare_ble_for_sync(self):
        # Avoid concurrent BLE scans from the live tab during sync/connect.
        self.exg_live_tab.stop_scan()

    def closeEvent(self, event):
        if hasattr(self, "exg_live_tab"):
            self.exg_live_tab.shutdown()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = OpenEarableToolApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
