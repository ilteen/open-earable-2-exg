import asyncio
from bleak import BleakClient, BleakScanner
import struct
from datetime import datetime
import threading
from collections import deque
import digitalfilter
import sys
import signal
import os
import argparse

import matplotlib.pyplot as plt
import matplotlib.animation as animation


BLE_ADDRESS = "E846C7ED-E084-6CAC-814C-23EA65F1B224"
SENSOR_CONFIG_CHAR_UUID = "34c2e3be-34aa-11eb-adc1-0242ac120002"
SENSOR_DATA_CHAR_UUID = "34c2e3bc-34aa-11eb-adc1-0242ac120002"
SENSOR_ID_EXG = 9
DATA_STREAMING = 0x01
EXG_SAMPLE_RATE_INDEX = 5  # 256 Hz in current firmware mapping (src/SensorManager/ExG.cpp)
EXG_SAMPLE_RATE_HZ = 256

SENSOR_CONFIG_FORMAT = "<BBB"
SENSOR_PACKET_HEADER_FORMAT = "<BBQ"
SENSOR_PACKET_HEADER_SIZE = struct.calcsize(SENSOR_PACKET_HEADER_FORMAT)
BLE_SCAN_TIMEOUT_SECONDS = 30.0
BLE_CONNECT_ATTEMPTS = 5
BLE_CONNECT_RETRY_DELAY_SECONDS = 1.0

# Plotting configuration
dataList = deque(maxlen=500)
max_datapoints_to_display = 500    # * 2 # Werte auf der x Achse
min_buffer_uV = 400                 # Skala der y Achse
sample_rate = EXG_SAMPLE_RATE_HZ
filters = digitalfilter.get_Biopotential_filter(order=4, cutoff=[1, 30], btype="bandpass", fs=sample_rate, output="sos")
#filters = digitalfilter.get_Biopotential_filter(order=4, cutoff=[1, 35], btype="bandpass", fs=256, output="sos") #EMG
#filters = digitalfilter.get_Biopotential_filter(order=4, cutoff= [1, 40], btype="bandpass", fs=256, output="sos") #ECG
#filters = digitalfilter.get_Biopotential_filter(order=4, cutoff=[1, 20], btype="bandpass", fs=256, output="sos") #ECG
#filters = digitalfilter.get_Biopotential_filter(order=4, cutoff=[5, 25], btype="bandpass", fs=256, output="sos") #ECG
enable_filters = True
write_to_file = False
autoscale = False

current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"ecg_lear_lfinger_{current_time}.csv"

# Global variables for cleanup
recording_file = None
is_shutting_down = False
ble_thread = None
data_lock = threading.Lock()

if write_to_file:
    recording_file = open("/Users/philipp/Library/Mobile Documents/com~apple~CloudDocs/KIT/Master/Masterarbeit/Code/Python Code/recordings/" + filename, 'w') #Ordnerpfad anpassen
    recording_file.write("time,raw_data,filtered_data\n")

fig, ax = plt.subplots()
line, = ax.plot([], [])

exit_event = threading.Event()

last_valid_timestamp = None
ble_target_uuid = BLE_ADDRESS


def build_sensor_config(sensor_id, sample_rate_index, storage_options):
    return struct.pack(SENSOR_CONFIG_FORMAT, sensor_id, sample_rate_index, storage_options)


def parse_sensor_packet(data):
    if len(data) < SENSOR_PACKET_HEADER_SIZE:
        return None
    sensor_id, payload_size, timestamp_us = struct.unpack(
        SENSOR_PACKET_HEADER_FORMAT, data[:SENSOR_PACKET_HEADER_SIZE]
    )
    end = SENSOR_PACKET_HEADER_SIZE + payload_size
    if end > len(data):
        return None
    return sensor_id, payload_size, timestamp_us, data[SENSOR_PACKET_HEADER_SIZE:end]


def looks_like_openearable(name):
    if not name:
        return False
    n = name.lower()
    return ("openearable" in n) or ("open-earable" in n) or ("open earable" in n)


async def resolve_device(target_uuid):
    """
    Resolve a connectable Bleak target.
    1) Prefer exact UUID/address match from scan.
    2) Fallback: if exactly one OpenEarable is visible, use it.
    """
    discovered = await BleakScanner.discover(timeout=BLE_SCAN_TIMEOUT_SECONDS, return_adv=True)
    target_norm = target_uuid.strip().lower()
    openearable_candidates = []

    for address, (device, adv) in discovered.items():
        device_addr = (device.address or "").lower()
        if address.lower() == target_norm or device_addr == target_norm:
            return device

        name = device.name or getattr(adv, "local_name", None)
        if looks_like_openearable(name):
            openearable_candidates.append(device)

    if len(openearable_candidates) == 1:
        picked = openearable_candidates[0]
        if target_norm and picked.address.lower() != target_norm:
            print(
                f"Requested UUID {target_uuid} not seen; using discovered OpenEarable {picked.address}."
            )
        return picked

    return None


async def stream_exg_with_client(client):
    start_cfg = build_sensor_config(SENSOR_ID_EXG, EXG_SAMPLE_RATE_INDEX, DATA_STREAMING)
    await client.write_gatt_char(SENSOR_CONFIG_CHAR_UUID, start_cfg)

    await client.start_notify(SENSOR_DATA_CHAR_UUID, notification_handler)
    print("Connected and receiving EXG data...")

    while not exit_event.is_set():
        await asyncio.sleep(0.1)  # Reduced sleep time for more responsive shutdown

    await client.stop_notify(SENSOR_DATA_CHAR_UUID)
    stop_cfg = build_sensor_config(SENSOR_ID_EXG, 0, 0)
    await client.write_gatt_char(SENSOR_CONFIG_CHAR_UUID, stop_cfg)

def init():
    line.set_data([], [])
    ax.set_xlim(0, max_datapoints_to_display)
    ax.set_title("Biopotential Data from OpenEarable ExG")
    ax.set_ylabel("Voltage (µV)")
    ax.set_xlabel("Samples")
    return line,

def animate(frame):
    global is_shutting_down
    
    # Don't animate if we're shutting down
    if is_shutting_down:
        return line,

    with data_lock:
        data = list(dataList)
    line.set_data(range(1, len(data) + 1), data)

    if data:
        min_val = min(data)
        max_val = max(data)
        buffer = 0.1 * (max_val - min_val) if max_val - min_val > min_buffer_uV else min_buffer_uV
        if autoscale:
            ax.set_ylim(min_val - buffer, max_val + buffer)
        else:
            ax.set_ylim(-min_buffer_uV, min_buffer_uV)
    
    return line,

def notification_handler(sender, data):
    global enable_filters, sample_rate, last_valid_timestamp, is_shutting_down
    
    # Don't process data if we're shutting down
    if is_shutting_down:
        return

    packet = parse_sensor_packet(data)
    if packet is None:
        return

    sensor_id, payload_size, _, payload = packet
    if sensor_id != SENSOR_ID_EXG or payload_size < 4:
        return

    float_value = struct.unpack("<f", payload[:4])[0]
    timestamp = datetime.now()
    filtered_data = float(filters(float_value))
    raw_data = float_value

    with data_lock:
        if enable_filters:
            dataList.append(filtered_data)
        else:
            dataList.append(raw_data)

    if write_to_file and recording_file and not is_shutting_down:
        try:
            recording_file.write(f"{timestamp.strftime('%H:%M:%S.%f')},{raw_data},{filtered_data}\n")
        except:
            pass  # Ignore write errors during shutdown

def insert_datapoint():
    global enable_filters, sample_rate, is_shutting_down
    
    # Don't insert data if we're shutting down
    if is_shutting_down:
        return

    timestamp = datetime.now()
    timestamp_for_float_value = timestamp.strftime("%H:%M:%S.%f")

    filtered_data = 1000000
    raw_data = 1000000

    with data_lock:
        if enable_filters:
            dataList.append(filtered_data)
        else:
            dataList.append(raw_data)

    if write_to_file and recording_file and not is_shutting_down:
        try:
            recording_file.write(f"{timestamp_for_float_value},{raw_data},{filtered_data}\n")
        except:
            pass  # Ignore write errors during shutdown

async def run_ble_client():
    last_error = None
    try:
        for attempt in range(1, BLE_CONNECT_ATTEMPTS + 1):
            if exit_event.is_set():
                return

            # First try direct connect to the provided UUID/address.
            try:
                print(f"Connect attempt {attempt}/{BLE_CONNECT_ATTEMPTS} to {ble_target_uuid}...")
                async with BleakClient(ble_target_uuid, timeout=10.0) as client:
                    await stream_exg_with_client(client)
                    return
            except Exception as e:
                last_error = e
                print(f"Direct connect failed: {e}")

            if exit_event.is_set():
                return

            # Fallback: scan and resolve current peripheral object.
            try:
                print("Scanning to resolve OpenEarable device...")
                device = await resolve_device(ble_target_uuid)
                if device is None:
                    print("No matching OpenEarable found in scan.")
                else:
                    print(f"Resolved device: {device.name} ({device.address})")
                    async with BleakClient(device, timeout=10.0) as client:
                        await stream_exg_with_client(client)
                        return
            except Exception as e:
                last_error = e
                print(f"Scan fallback failed: {e}")

            await asyncio.sleep(BLE_CONNECT_RETRY_DELAY_SECONDS)

        if last_error is not None:
            raise RuntimeError(
                f"Failed to connect after {BLE_CONNECT_ATTEMPTS} attempts: {last_error}"
            )
        raise RuntimeError(f"Failed to connect after {BLE_CONNECT_ATTEMPTS} attempts.")
    except Exception as e:
        if not is_shutting_down:
            print(f"BLE connection error: {e}")

def start_async_loop():
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(run_ble_client())
    except Exception as e:
        if not is_shutting_down:
            print(f"Async loop error: {e}")

def cleanup(*args):
    global is_shutting_down, recording_file, ble_thread
    if is_shutting_down:
        return

    is_shutting_down = True
    exit_event.set()

    # Allow BLE worker to stop notifications and send stop command.
    if ble_thread and ble_thread.is_alive():
        ble_thread.join(timeout=3.0)
    
    if recording_file:
        try:
            recording_file.close()
            recording_file = None
        except:
            pass
    
    try:
        plt.close('all')
    except:
        pass

def handle_close(evt):
    cleanup()

def handle_key_press(event):
    if event.key == 'g' and not is_shutting_down:
        insert_datapoint()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OpenEarable EXG live plot over BLE")
    parser.add_argument(
        "--uuid",
        default=BLE_ADDRESS,
        help="Device UUID/address to auto-connect (default: %(default)s)",
    )
    args = parser.parse_args()
    ble_target_uuid = args.uuid.strip()

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    fig.canvas.mpl_connect('close_event', handle_close)
    fig.canvas.mpl_connect('key_press_event', handle_key_press)
    
    ble_thread = threading.Thread(target=start_async_loop, daemon=False)
    ble_thread.start()

    try:
        ani = animation.FuncAnimation(fig, animate, init_func=init, interval=20, save_count=max_datapoints_to_display)
        plt.show()
    except KeyboardInterrupt:
        cleanup()
    except Exception as e:
        print(f"Application error: {e}")
        cleanup()
    finally:
        cleanup()
