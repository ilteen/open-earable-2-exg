#!/usr/bin/env python3
"""Test BLE scanning to debug OpenEarable discovery issues."""

import asyncio
from bleak import BleakScanner

# Known OpenEarable service UUIDs
TIME_SYNC_SERVICE_UUID = "2e04cbf7-939d-4be5-823e-271838b75259"


async def test_scan():
    print("Starting detailed BLE scan...")
    print("Looking for OpenEarable devices (8 second scan)...\n")
    
    devices_dict = {}
    
    def callback(device, adv_data):
        # Store with all available info, update if we get more info
        addr = device.address
        name = device.name or adv_data.local_name
        
        # Update or add device
        if addr not in devices_dict or name:
            devices_dict[addr] = {
                'device': device,
                'adv_data': adv_data,
                'name': name,
                'services': adv_data.service_uuids,
                'rssi': adv_data.rssi,
            }
    
    scanner = BleakScanner(detection_callback=callback)
    await scanner.start()
    await asyncio.sleep(8.0)
    await scanner.stop()
    
    print(f"Found {len(devices_dict)} total devices\n")
    
    # Check for OpenEarable by name
    print("=== Devices with 'OpenEarable' in name ===")
    openearable_by_name = [(addr, info) for addr, info in devices_dict.items() 
                          if info['name'] and 'OpenEarable' in info['name']]
    if openearable_by_name:
        for addr, info in openearable_by_name:
            print(f"  {info['name']}: {addr} (RSSI: {info['rssi']})")
            if info['services']:
                print(f"    Services: {info['services']}")
    else:
        print("  None found")
    
    # Check for OpenEarable by service UUID
    print("\n=== Devices with Time Sync Service UUID ===")
    openearable_by_service = [(addr, info) for addr, info in devices_dict.items()
                              if TIME_SYNC_SERVICE_UUID.lower() in [s.lower() for s in info['services']]]
    if openearable_by_service:
        for addr, info in openearable_by_service:
            print(f"  {info['name'] or 'Unknown'}: {addr} (RSSI: {info['rssi']})")
    else:
        print("  None found")
    
    # Show all named devices for debugging
    print("\n=== All devices with names ===")
    named_devices = [(addr, info) for addr, info in devices_dict.items() if info['name']]
    for addr, info in sorted(named_devices, key=lambda x: -(x[1]['rssi'] or -100)):
        print(f"  {info['name']}: {addr} (RSSI: {info['rssi']})")


if __name__ == "__main__":
    asyncio.run(test_scan())
