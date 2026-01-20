#!/usr/bin/env python3
"""
GUI tool for OpenEarable v2:
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

import asyncio
import os
import struct
import sys
import threading
import time
from collections import defaultdict
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

# Try PyQt6 first, fall back to PySide6 (better Windows compatibility)
try:
    from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QLabel, QTableWidget, QTableWidgetItem, QProgressBar,
        QMessageBox, QHeaderView, QGroupBox, QTabWidget, QFileDialog,
        QListWidget, QTextEdit, QSplitter, QCheckBox
    )
    from PyQt6.QtGui import QTextCursor, QFont
    QT_BACKEND = "PyQt6"
except ImportError:
    try:
        from PySide6.QtCore import Qt, QTimer, Signal as pyqtSignal, QObject
        from PySide6.QtWidgets import (
            QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
            QPushButton, QLabel, QTableWidget, QTableWidgetItem, QProgressBar,
            QMessageBox, QHeaderView, QGroupBox, QTabWidget, QFileDialog,
            QListWidget, QTextEdit, QSplitter, QCheckBox
        )
        from PySide6.QtGui import QTextCursor, QFont
        QT_BACKEND = "PySide6"
    except ImportError:
        print("Error: Neither PyQt6 nor PySide6 is installed.")
        print("Please install one of them:")
        print("  pip install PyQt6")
        print("  pip install PySide6  (recommended for Windows)")
        sys.exit(1)

import pandas as pd
import numpy as np

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


async def scan_devices(timeout: float = 5.0) -> list[BLEDevice]:
    devices = await BleakScanner.discover(timeout=timeout)
    openearable = [d for d in devices if d.name and "OpenEarable" in d.name]
    return sorted(openearable, key=lambda d: -(d.rssi or -100))


async def sync_time(device: BLEDevice, progress_callback=None) -> tuple[bool, str]:
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

    try:
        async with BleakClient(device) as client:
            if progress_callback:
                progress_callback(f"Connected to {device.name}")

            await client.start_notify(TIME_SYNC_RTT_CHAR_UUID, notification_handler)

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

            await client.stop_notify(TIME_SYNC_RTT_CHAR_UUID)

            if len(offsets) < 1:
                return False, "No valid samples collected!"

            median_offset = compute_median(offsets)
            offset_bytes = struct.pack("<q", median_offset)
            await client.write_gatt_char(TIME_SYNC_OFFSET_CHAR_UUID, offset_bytes)

            offset_sec = median_offset / 1_000_000
            return True, f"Time synced!\n\nOffset: {offset_sec:+.3f} seconds\nSamples: {len(offsets)}/{TIME_SYNC_SAMPLES}"

    except Exception as e:
        return False, f"Error: {str(e)}"


# =============================================================================
# USB Time Sync
# =============================================================================

# USB Protocol constants (must match firmware usb_time_sync.c)
USB_SYNC_MAGIC = 0xAA
USB_SYNC_REQUEST = 0x01
USB_SYNC_RESPONSE = 0x02
USB_SYNC_OFFSET = 0x03

USB_REQUEST_SIZE = 11   # magic(1) + op(1) + seq(1) + t1(8)
USB_RESPONSE_SIZE = 27  # magic(1) + op(1) + seq(1) + t1(8) + t2(8) + t3(8)
USB_OFFSET_SIZE = 10    # magic(1) + op(1) + offset(8)

USB_TIME_SYNC_SAMPLES = 10  # More samples for USB since it's faster


def find_openearable_usb_ports() -> list[tuple[str, str, str]]:
    """Find USB serial ports that are OpenEarable devices.
    
    Returns list of (port, description, hwid) tuples.
    Only returns devices with "OpenEarable" in the product/description.
    """
    if not SERIAL_AVAILABLE:
        return []
    
    ports = []
    for port in serial.tools.list_ports.comports():
        # Only match devices that have "OpenEarable" in description or product
        description = port.description or ""
        product = port.product or ""
        manufacturer = port.manufacturer or ""
        
        is_openearable = (
            "OpenEarable" in description or
            "OpenEarable" in product or
            "OpenEarable" in manufacturer
        )
        
        if is_openearable:
            display_name = product if product else description
            ports.append((port.device, display_name, port.hwid or ""))
    return ports


def create_usb_request_packet(seq: int, t1_us: int) -> bytes:
    """Create USB time sync request packet."""
    return struct.pack('<BBBq', USB_SYNC_MAGIC, USB_SYNC_REQUEST, seq, t1_us)


def create_usb_offset_packet(offset_us: int) -> bytes:
    """Create USB time sync offset packet."""
    return struct.pack('<BBq', USB_SYNC_MAGIC, USB_SYNC_OFFSET, offset_us)


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


def sync_time_usb(port: str, progress_callback=None, debug_callback=None) -> tuple[bool, str]:
    """
    Perform time sync over USB serial.
    
    Args:
        port: Serial port path (e.g., /dev/tty.usbmodem*)
        progress_callback: Called with progress messages
        debug_callback: Called with debug info (tx/rx bytes, timing)
    
    Returns:
        (success, message) tuple
    """
    if not SERIAL_AVAILABLE:
        return False, "pyserial not installed. Run: pip install pyserial"
    
    offsets: list[int] = []
    rtts: list[int] = []
    
    def debug(msg: str):
        if debug_callback:
            debug_callback(msg)
    
    try:
        debug(f"Opening serial port: {port}")
        with serial.Serial(port, 115200, timeout=2.0) as ser:
            if progress_callback:
                progress_callback(f"Connected to {port}")
            debug(f"Serial port opened successfully")
            
            # Clear any pending data
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            debug("Buffers cleared, starting sync samples...")
            
            for i in range(USB_TIME_SYNC_SAMPLES):
                if progress_callback:
                    progress_callback(f"Collecting sample {i + 1}/{USB_TIME_SYNC_SAMPLES}...")
                
                # Get current time in microseconds
                t1 = int(time.time() * 1_000_000)
                
                # Create and send request packet
                request = create_usb_request_packet(i, t1)
                debug(f"\n[TX] Sample {i+1}: magic=0xAA op=0x01 seq={i} t1={t1}")
                debug(f"     Raw: {request.hex()}")
                
                ser.write(request)
                ser.flush()
                
                # Read response
                response = ser.read(USB_RESPONSE_SIZE)
                t4 = int(time.time() * 1_000_000)
                
                if len(response) == USB_RESPONSE_SIZE:
                    debug(f"[RX] Got {len(response)} bytes: {response.hex()}")
                    
                    pkt = parse_usb_response_packet(response)
                    if pkt:
                        t2 = pkt["t2"]
                        t3 = pkt["t3"]
                        
                        # Calculate RTT and offset using NTP algorithm
                        # RTT = (t4 - t1) - (t3 - t2)
                        # offset = ((t2 - t1) + (t3 - t4)) / 2
                        rtt = (t4 - t1) - (t3 - t2)
                        offset = ((t2 - t1) + (t3 - t4)) // 2
                        
                        debug(f"     t1={t1}, t2={t2}, t3={t3}, t4={t4}")
                        debug(f"     RTT={rtt}µs ({rtt/1000:.3f}ms), offset={offset}µs")
                        
                        offsets.append(offset)
                        rtts.append(rtt)
                    else:
                        debug(f"     Failed to parse response!")
                else:
                    debug(f"[RX] Timeout or incomplete: got {len(response)} bytes (expected {USB_RESPONSE_SIZE})")
                    if response:
                        debug(f"     Raw: {response.hex()}")
                
                time.sleep(0.02)  # Short delay between samples
            
            if len(offsets) < 1:
                return False, "No valid samples collected!"
            
            # Calculate median offset (more robust than mean)
            median_offset = compute_median(offsets)
            avg_rtt = sum(rtts) / len(rtts)
            
            debug(f"\n=== Results ===")
            debug(f"Valid samples: {len(offsets)}/{USB_TIME_SYNC_SAMPLES}")
            debug(f"Offsets: {offsets}")
            debug(f"Median offset: {median_offset}µs ({median_offset/1_000_000:.6f}s)")
            debug(f"Average RTT: {avg_rtt:.0f}µs ({avg_rtt/1000:.3f}ms)")
            
            # Send offset to device
            offset_packet = create_usb_offset_packet(median_offset)
            debug(f"\n[TX] Sending offset: {median_offset}µs")
            debug(f"     Raw: {offset_packet.hex()}")
            
            ser.write(offset_packet)
            ser.flush()
            
            offset_sec = median_offset / 1_000_000
            rtt_ms = avg_rtt / 1000
            
            return True, (
                f"Time synced via USB!\n\n"
                f"Offset: {offset_sec:+.6f} seconds\n"
                f"Average RTT: {rtt_ms:.3f} ms\n"
                f"Samples: {len(offsets)}/{USB_TIME_SYNC_SAMPLES}"
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


def parse_oe_file(filename: str) -> tuple[pd.DataFrame, dict]:
    """Parse .oe file and return DataFrame and metadata."""
    FILE_HEADER_FORMAT = '<HQ'
    FILE_HEADER_SIZE = struct.calcsize(FILE_HEADER_FORMAT)
    
    data = defaultdict(list)
    metadata = {}
    _sid = None

    with open(filename, 'rb') as f:
        version, timestamp = struct.unpack(FILE_HEADER_FORMAT, f.read(FILE_HEADER_SIZE))
        metadata['version'] = version
        metadata['timestamp'] = timestamp
        metadata['filename'] = os.path.basename(filename)

        while True:
            header = f.read(10)
            if len(header) < 10:
                break
            sid, size, time_us = struct.unpack('<BBQ', header)
            if size > 192 or sid > 9:
                if _sid is not None and _sid in data.keys():
                    data[_sid].pop()
                break

            _sid = sid
            raw_data = f.read(size)
            if len(raw_data) < size:
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
    scan_complete = pyqtSignal(list)
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
        self.loop = asyncio.new_event_loop()
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

        # Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.start_scan)
        btn_layout.addWidget(self.refresh_btn)

        self.sync_btn = QPushButton("⏱ Sync Time")
        self.sync_btn.clicked.connect(self.start_sync)
        btn_layout.addWidget(self.sync_btn)

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
        self.progress.setVisible(not enabled)

    def start_scan(self):
        self.set_buttons_enabled(False)
        self.status_label.setText("Scanning for devices...")
        self.table.setRowCount(0)
        thread = threading.Thread(target=self._scan_thread, daemon=True)
        thread.start()

    def _scan_thread(self):
        try:
            devices = self.loop.run_until_complete(scan_devices(timeout=5.0))
            self.signals.scan_complete.emit(devices)
        except Exception as e:
            self.signals.scan_error.emit(str(e))

    def on_scan_complete(self, devices: list[BLEDevice]):
        self.devices = devices
        self.set_buttons_enabled(True)

        self.table.setRowCount(len(devices))
        for i, device in enumerate(devices):
            self.table.setItem(i, 0, QTableWidgetItem(device.name or "Unknown"))
            self.table.setItem(i, 1, QTableWidgetItem(str(device.rssi or "N/A")))
            self.table.setItem(i, 2, QTableWidgetItem(device.address))

        if devices:
            self.status_label.setText(f"Found {len(devices)} device(s). Select one and click Sync.")
            self.table.selectRow(0)
        else:
            self.status_label.setText("No OpenEarable devices found. Click Refresh to scan again.")

    def on_scan_error(self, error: str):
        self.set_buttons_enabled(True)
        self.status_label.setText(f"Scan error: {error}")
        QMessageBox.critical(self, "Scan Error", error)

    def start_sync(self):
        selected = self.table.selectedItems()
        if not selected:
            QMessageBox.warning(self, "No Selection", "Please select a device first.")
            return

        row = self.table.currentRow()
        if row < 0 or row >= len(self.devices):
            return

        device = self.devices[row]
        self.set_buttons_enabled(False)
        self.status_label.setText(f"Connecting to {device.name}...")

        thread = threading.Thread(target=self._sync_thread, args=(device,), daemon=True)
        thread.start()

    def _sync_thread(self, device: BLEDevice):
        def progress_cb(msg):
            self.signals.progress.emit(msg)

        try:
            success, message = self.loop.run_until_complete(sync_time(device, progress_cb))
            self.signals.sync_complete.emit(success, message)
        except Exception as e:
            self.signals.sync_complete.emit(False, str(e))

    def on_progress(self, msg: str):
        self.status_label.setText(msg)

    def on_sync_complete(self, success: bool, message: str):
        self.set_buttons_enabled(True)
        if success:
            self.status_label.setText("✓ Time sync successful!")
            QMessageBox.information(self, "Success", message)
        else:
            self.status_label.setText("✗ Time sync failed")
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

        # Create splitter for port list and debug console
        splitter = QSplitter(Qt.Orientation.Vertical)
        
        # Top part: Port selection
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        
        # Port list group
        group = QGroupBox("USB Serial Ports")
        group_layout = QVBoxLayout(group)

        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Port", "Description", "Hardware ID"])
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setMaximumHeight(120)
        group_layout.addWidget(self.table)
        top_layout.addWidget(group)

        # Status and buttons row
        status_btn_layout = QHBoxLayout()
        
        self.status_label = QLabel("Click Refresh to scan for USB ports...")
        self.status_label.setStyleSheet("color: gray;")
        status_btn_layout.addWidget(self.status_label, stretch=1)
        
        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        status_btn_layout.addWidget(self.refresh_btn)

        self.sync_btn = QPushButton("⏱ Sync Time (USB)")
        self.sync_btn.clicked.connect(self.start_sync)
        status_btn_layout.addWidget(self.sync_btn)
        
        top_layout.addLayout(status_btn_layout)

        # Progress bar
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setVisible(False)
        top_layout.addWidget(self.progress)
        
        splitter.addWidget(top_widget)
        
        # Bottom part: Debug console
        debug_widget = QWidget()
        debug_layout = QVBoxLayout(debug_widget)
        debug_layout.setContentsMargins(0, 0, 0, 0)
        
        debug_header = QHBoxLayout()
        debug_label = QLabel("🔧 Debug Console")
        debug_label.setStyleSheet("font-weight: bold;")
        debug_header.addWidget(debug_label)
        
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
        debug_layout.addWidget(self.debug_console)
        
        splitter.addWidget(debug_widget)
        
        # Set initial splitter sizes (60% top, 40% bottom)
        splitter.setSizes([300, 200])
        
        layout.addWidget(splitter)

        # Info box at bottom
        if not SERIAL_AVAILABLE:
            info = QLabel("⚠️ pyserial not installed. Run: pip install pyserial")
            info.setStyleSheet("color: #ff6b6b; font-size: 12px; padding: 5px;")
        else:
            info = QLabel("💡 USB sync is faster and more accurate than Bluetooth (~1ms vs ~10ms).\n"
                          "Connect OpenEarable via USB cable and select the port.")
            info.setStyleSheet("color: #666; font-size: 11px;")
        info.setWordWrap(True)
        layout.addWidget(info)

    def _append_debug(self, msg: str):
        """Append message to debug console (thread-safe via signal)."""
        self.debug_console.append(msg)
        if self.auto_scroll_cb.isChecked():
            cursor = self.debug_console.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self.debug_console.setTextCursor(cursor)

    def _clear_debug(self):
        """Clear the debug console."""
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
        self.progress.setVisible(not enabled)

    def start_sync(self):
        """Start USB time sync."""
        selected = self.table.selectedItems()
        if not selected:
            QMessageBox.warning(self, "No Selection", "Please select a port first.")
            return

        row = self.table.currentRow()
        if row < 0 or row >= len(self.ports):
            return

        port = self.ports[row][0]
        self.set_buttons_enabled(False)
        self.status_label.setText(f"Syncing time via {port}...")
        
        self._append_debug(f"\n{'='*50}")
        self._append_debug(f"Starting USB time sync on {port}")
        self._append_debug(f"{'='*50}")

        thread = threading.Thread(target=self._sync_thread, args=(port,), daemon=True)
        thread.start()

    def _sync_thread(self, port: str):
        """Background thread for USB sync."""
        def progress_cb(msg):
            self.signals.progress.emit(msg)

        def debug_cb(msg):
            # Use signal for thread-safe GUI update
            self.debug_message.emit(msg)

        try:
            success, message = sync_time_usb(port, progress_cb, debug_cb)
            self.signals.sync_complete.emit(success, message)
        except Exception as e:
            self.debug_message.emit(f"EXCEPTION: {e}")
            self.signals.sync_complete.emit(False, str(e))

    def on_progress(self, msg: str):
        self.status_label.setText(msg)

    def on_sync_complete(self, success: bool, message: str):
        self.set_buttons_enabled(True)
        if success:
            self.status_label.setText("✓ USB time sync successful!")
            self._append_debug(f"\n✓ SUCCESS!")
            QMessageBox.information(self, "Success", message)
        else:
            self.status_label.setText("✗ USB time sync failed")
            self._append_debug(f"\n✗ FAILED: {message}")
            QMessageBox.critical(self, "Sync Failed", message)


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

        # File list group
        group = QGroupBox("Loaded .oe Files")
        group_layout = QVBoxLayout(group)

        self.file_list = QListWidget()
        self.file_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        self.file_list.itemSelectionChanged.connect(self._on_selection_changed)
        group_layout.addWidget(self.file_list)

        layout.addWidget(group)

        # Info display
        info_group = QGroupBox("File Info")
        info_layout = QVBoxLayout(info_group)
        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(120)
        info_layout.addWidget(self.info_text)
        layout.addWidget(info_group)

        # Status label
        self.status_label = QLabel("Import .oe files to convert to CSV")
        self.status_label.setStyleSheet("color: gray;")
        layout.addWidget(self.status_label)

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
        layout.addLayout(btn_layout)

        # Progress bar
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setVisible(False)
        layout.addWidget(self.progress)

    def _on_selection_changed(self):
        selected = self.file_list.selectedItems()
        if len(selected) == 1:
            idx = self.file_list.row(selected[0])
            if idx < len(self.loaded_files):
                _, df, metadata = self.loaded_files[idx]
                info = f"📄 {metadata['filename']}\n"
                info += f"Version: {metadata['version']}\n"
                info += f"Rows: {len(df)}\n\n"
                info += "Sensor samples:\n"
                for sensor, count in metadata['sensor_counts'].items():
                    if count > 0:
                        info += f"  • {sensor}: {count}\n"
                self.info_text.setText(info)
        elif len(selected) > 1:
            self.info_text.setText(f"{len(selected)} files selected")
        else:
            self.info_text.clear()

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
        self.info_text.clear()
        self.export_btn.setEnabled(False)
        self.clear_btn.setEnabled(False)
        self.status_label.setText("Import .oe files to convert to CSV")


class OpenEarableToolApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenEarable Tools")
        self.setMinimumSize(700, 600)

        self.signals = WorkerSignals()
        self._create_widgets()
        self._connect_signals()

        # Auto-scan on startup
        QTimer.singleShot(100, self.time_sync_tab.start_scan)
        QTimer.singleShot(200, self.usb_sync_tab.refresh_ports)

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
        
        self.time_sync_tab = TimeSyncTab(self.signals)
        self.tabs.addTab(self.time_sync_tab, "📶 BLE Time Sync")

        self.usb_sync_tab = USBTimeSyncTab(self.signals)
        self.tabs.addTab(self.usb_sync_tab, "🔌 USB Time Sync")

        self.file_tab = FileConverterTab(self.signals)
        self.tabs.addTab(self.file_tab, "📂 File Converter")

        layout.addWidget(self.tabs)

    def _connect_signals(self):
        # BLE Time sync signals
        self.signals.scan_complete.connect(self.time_sync_tab.on_scan_complete)
        self.signals.scan_error.connect(self.time_sync_tab.on_scan_error)
        self.signals.sync_complete.connect(self._on_sync_complete)
        self.signals.progress.connect(self._on_progress)

        # File converter signals
        self.signals.parse_complete.connect(self.file_tab.on_parse_complete)
        self.signals.parse_error.connect(self.file_tab.on_parse_error)

    def _on_sync_complete(self, success: bool, message: str):
        """Route sync complete signal to the active tab."""
        current = self.tabs.currentWidget()
        if isinstance(current, TimeSyncTab):
            current.on_sync_complete(success, message)
        elif isinstance(current, USBTimeSyncTab):
            current.on_sync_complete(success, message)

    def _on_progress(self, msg: str):
        """Route progress signal to the active tab."""
        current = self.tabs.currentWidget()
        if hasattr(current, 'on_progress'):
            current.on_progress(msg)


def main():
    app = QApplication(sys.argv)
    window = OpenEarableToolApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
