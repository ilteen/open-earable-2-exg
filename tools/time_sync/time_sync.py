#!/usr/bin/env python3
"""
Simple Python script to sync time with OpenEarable v2 devices via BLE.

The script will:
1. Scan for BLE devices
2. Let you choose an OpenEarable device
3. Connect and synchronize time (same protocol as the app)
"""

import asyncio
import struct
import time
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice


# UUIDs from the firmware
TIME_SYNC_SERVICE_UUID = "2e04cbf7-939d-4be5-823e-271838b75259"
TIME_SYNC_OFFSET_CHAR_UUID = "2e04cbf8-939d-4be5-823e-271838b75259"  # Write offset
TIME_SYNC_RTT_CHAR_UUID = "2e04cbf9-939d-4be5-823e-271838b75259"     # Write request, notify response

# Time sync packet structure
# version: 1 byte
# op: 1 byte (0=request, 1=response)
# seq: 2 bytes (little endian)
# t1_phone: 8 bytes (little endian, microseconds)
# t2_dev_rx: 8 bytes (little endian, microseconds)
# t3_dev_tx: 8 bytes (little endian, microseconds)
PACKET_FORMAT = "<BBHqqq"  # 28 bytes total
PACKET_SIZE = 28

TIME_SYNC_SAMPLES = 7


def create_request_packet(seq: int, t1_us: int) -> bytes:
    """Create a time sync request packet."""
    return struct.pack(
        PACKET_FORMAT,
        1,       # version
        0,       # op = request
        seq,     # sequence number
        t1_us,   # phone send time
        0,       # device receive time (filled by device)
        0,       # device send time (filled by device)
    )


def parse_response_packet(data: bytes) -> Optional[dict]:
    """Parse a time sync response packet."""
    if len(data) < PACKET_SIZE:
        print(f"  Warning: Packet too short ({len(data)} bytes)")
        return None
    
    version, op, seq, t1, t2, t3 = struct.unpack(PACKET_FORMAT, data[:PACKET_SIZE])
    
    if op != 1:  # Not a response
        return None
    
    return {
        "version": version,
        "op": op,
        "seq": seq,
        "t1_phone_send": t1,
        "t2_device_rx": t2,
        "t3_device_tx": t3,
    }


def compute_median(values: list[int]) -> int:
    """Compute median of a list of integers."""
    sorted_vals = sorted(values)
    mid = len(sorted_vals) // 2
    if len(sorted_vals) % 2 == 1:
        return sorted_vals[mid]
    else:
        return (sorted_vals[mid - 1] + sorted_vals[mid]) // 2


async def scan_devices(timeout: float = 5.0) -> list[BLEDevice]:
    """Scan for BLE devices."""
    print(f"\nScanning for BLE devices ({timeout}s)...")
    devices = await BleakScanner.discover(timeout=timeout)
    return devices


async def sync_time(device: BLEDevice) -> None:
    """Connect to device and synchronize time."""
    print(f"\nConnecting to {device.name} ({device.address})...")
    
    offsets: list[int] = []
    response_event = asyncio.Event()
    current_response: dict = {}
    
    def notification_handler(sender, data: bytearray):
        nonlocal current_response
        t4 = int(time.time() * 1_000_000)  # Current time in microseconds
        
        pkt = parse_response_packet(bytes(data))
        if pkt is None:
            return
        
        t1 = pkt["t1_phone_send"]
        t3 = pkt["t3_device_tx"]
        
        # Estimate Unix time at the moment the device sent the response
        # Use midpoint between T1 and T4
        unix_at_t3 = t1 + ((t4 - t1) // 2)
        
        # offset = unix_time - device_time
        offset = unix_at_t3 - t3
        
        current_response = {
            "t1": t1,
            "t3": t3,
            "t4": t4,
            "offset": offset,
        }
        response_event.set()
    
    async with BleakClient(device) as client:
        print(f"Connected! MTU: {client.mtu_size}")
        
        # Subscribe to RTT notifications
        await client.start_notify(TIME_SYNC_RTT_CHAR_UUID, notification_handler)
        print("Subscribed to RTT notifications")
        
        # Send RTT requests and collect samples
        print(f"\nCollecting {TIME_SYNC_SAMPLES} time sync samples...")
        
        for i in range(TIME_SYNC_SAMPLES):
            t1 = int(time.time() * 1_000_000)  # Current time in microseconds
            request = create_request_packet(i, t1)
            
            response_event.clear()
            await client.write_gatt_char(TIME_SYNC_RTT_CHAR_UUID, request)
            
            try:
                await asyncio.wait_for(response_event.wait(), timeout=2.0)
                offset = current_response["offset"]
                offsets.append(offset)
                print(f"  Sample {i + 1}: offset = {offset:+,} µs ({offset / 1_000_000:+.3f} s)")
            except asyncio.TimeoutError:
                print(f"  Sample {i + 1}: timeout waiting for response")
            
            await asyncio.sleep(0.05)  # 50ms between requests
        
        # Stop notifications
        await client.stop_notify(TIME_SYNC_RTT_CHAR_UUID)
        
        if len(offsets) < 1:
            print("\nError: No valid samples collected!")
            return
        
        # Compute median offset
        median_offset = compute_median(offsets)
        print(f"\nMedian offset: {median_offset:,} µs ({median_offset / 1_000_000:.3f} s)")
        
        # Write the offset to the device
        offset_bytes = struct.pack("<q", median_offset)  # int64 little endian
        await client.write_gatt_char(TIME_SYNC_OFFSET_CHAR_UUID, offset_bytes)
        
        print("✓ Time offset written to device - sync complete!")
        print(f"\nDevice time is now synchronized to Unix epoch.")


async def main():
    print("=" * 50)
    print("OpenEarable v2 Time Sync Tool")
    print("=" * 50)
    
    # Scan for devices
    devices = await scan_devices()
    
    if not devices:
        print("No devices found!")
        return
    
    # Filter to only OpenEarable devices
    openearable_devices = [d for d in devices if d.name and "OpenEarable" in d.name]
    
    if not openearable_devices:
        print("No OpenEarable devices found!")
        return
    
    # Sort by signal strength
    openearable_devices = sorted(openearable_devices, key=lambda d: -(d.rssi or -100))
    
    print(f"\nFound {len(openearable_devices)} OpenEarable device(s):\n")
    
    for i, device in enumerate(openearable_devices):
        print(f"  [{i:2}] {device.name:30} RSSI: {device.rssi:4} dBm  {device.address}")
    
    devices = openearable_devices  # Use filtered list
    
    # Let user choose
    print("\nEnter device number (or 'q' to quit): ", end="")
    
    try:
        choice = input().strip()
        if choice.lower() == 'q':
            return
        
        idx = int(choice)
        if idx < 0 or idx >= len(devices):
            print("Invalid selection!")
            return
        
        selected = devices[idx]
        
    except (ValueError, EOFError):
        print("Invalid input!")
        return
    
    # Connect and sync
    try:
        await sync_time(selected)
    except Exception as e:
        print(f"\nError: {e}")
        raise


if __name__ == "__main__":
    asyncio.run(main())
