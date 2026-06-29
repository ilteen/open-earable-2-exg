import math
import mmap
import os
import struct
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Callable, Optional

import numpy as np
import pandas as pd

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

SID_SENSOR = {sid: name for name, sid in SENSOR_SID.items()}

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

SENSOR_PACKET_HEADER_FORMAT = "<BBQ"
SENSOR_PACKET_HEADER_SIZE = struct.calcsize(SENSOR_PACKET_HEADER_FORMAT)
MAX_SENSOR_PAYLOAD_SIZE = 192
PARSE_RESYNC_SCAN_BYTES = 512
PARSE_TARGET_CHUNK_SIZE = 8 * 1024 * 1024
PARSE_MIN_PARALLEL_FILE_SIZE = 64 * 1024 * 1024
MAX_PARALLEL_PARSE_WORKERS = 8


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


def _find_resync_offset_in_buffer(buffer, start_offset: int, stop_offset: int) -> Optional[int]:
    if start_offset + SENSOR_PACKET_HEADER_SIZE >= stop_offset:
        return None

    max_window_end = min(
        stop_offset,
        start_offset + PARSE_RESYNC_SCAN_BYTES + SENSOR_PACKET_HEADER_SIZE + MAX_SENSOR_PAYLOAD_SIZE + SENSOR_PACKET_HEADER_SIZE,
    )
    window = buffer[start_offset:max_window_end]

    for shift in range(1, min(PARSE_RESYNC_SCAN_BYTES, len(window) - SENSOR_PACKET_HEADER_SIZE) + 1):
        if shift + SENSOR_PACKET_HEADER_SIZE > len(window):
            break

        sensor_id, payload_size, _ = struct.unpack_from(SENSOR_PACKET_HEADER_FORMAT, window, shift)
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


def _parse_file_header(filename: str) -> tuple[dict, int, int]:
    file_header_format = '<HQ'
    file_header_size = struct.calcsize(file_header_format)
    file_header_v3_format = '<HQIIQB'
    file_header_v3_size = struct.calcsize(file_header_v3_format)

    file_size = os.path.getsize(filename)
    if file_size == 0:
        raise ValueError("File is empty (0 bytes). Recording did not contain data or was not finalized.")

    with open(filename, 'rb') as f:
        file_header = f.read(file_header_size)
        if len(file_header) < file_header_size:
            raise ValueError(
                f"File header is truncated (expected {file_header_size} bytes, got {len(file_header)})."
            )

        version, timestamp = struct.unpack(file_header_format, file_header)
        metadata = {
            'version': version,
            'timestamp': timestamp,
            'filename': os.path.basename(filename),
            'filesize_bytes': file_size,
            'truncated': False,
            'header_size': file_header_size,
            'resync_count': 0,
            'resync_positions': [],
        }
        data_offset = file_header_size

        if version == 0x0003:
            f.seek(0)
            file_header_v3 = f.read(file_header_v3_size)
            if len(file_header_v3) < file_header_v3_size:
                raise ValueError(
                    f"Version 0x0003 header is truncated (expected {file_header_v3_size} bytes, got {len(file_header_v3)})."
                )

            version, timestamp, header_size, parse_info_size, device_id, side = struct.unpack(
                file_header_v3_format, file_header_v3
            )
            if header_size < file_header_v3_size or header_size > file_size:
                raise ValueError(f"Invalid v0x0003 header size {header_size} for file size {file_size}.")

            metadata['timestamp'] = timestamp
            metadata['header_size'] = header_size
            metadata['parse_info_size'] = parse_info_size
            metadata['device_id'] = device_id
            metadata['side'] = side
            data_offset = header_size

    return metadata, data_offset, file_size


def _build_parse_ranges(data_offset: int, file_size: int, worker_count: int) -> list[tuple[int, int, bool, bool]]:
    if worker_count <= 1:
        return [(data_offset, file_size, True, True)]

    ranges = []
    start = data_offset
    index = 0
    while start < file_size:
        end = min(file_size, start + PARSE_TARGET_CHUNK_SIZE)
        ranges.append((start, end, index == 0, end >= file_size))
        start = end
        index += 1
    return ranges


def _empty_chunk_data() -> dict:
    chunk_data = {}
    for sensor_name in ACTIVE_SENSORS:
        sid = SENSOR_SID[sensor_name]
        chunk_data[sid] = {'timestamps': [], 'values': []}
    return chunk_data


def _parse_oe_chunk(args):
    filename, start_offset, end_offset, is_first_chunk, is_last_chunk, file_size = args
    chunk_data = _empty_chunk_data()
    metadata = {'truncated': False, 'resync_count': 0, 'resync_positions': []}

    with open(filename, 'rb') as f:
        with mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ) as mm:
            pos = start_offset

            if not is_first_chunk:
                resync_pos = _find_resync_offset_in_buffer(mm, start_offset - 1, file_size)
                if resync_pos is None:
                    return chunk_data, metadata
                pos = start_offset - 1 + resync_pos

            while pos + SENSOR_PACKET_HEADER_SIZE <= file_size:
                if not is_last_chunk and pos >= end_offset:
                    break

                sid, size, time_us = struct.unpack_from(SENSOR_PACKET_HEADER_FORMAT, mm, pos)
                if not _record_header_is_valid(sid, size):
                    resync_offset = _find_resync_offset_in_buffer(mm, pos, file_size)
                    if resync_offset is None:
                        metadata['truncated'] = True
                        break
                    metadata['resync_count'] += 1
                    metadata['resync_positions'].append(pos)
                    pos += resync_offset
                    continue

                payload_start = pos + SENSOR_PACKET_HEADER_SIZE
                payload_end = payload_start + size
                if payload_end > file_size:
                    metadata['truncated'] = True
                    break

                if sid in chunk_data and sid in SENSOR_FORMATS and sid not in (SENSOR_SID["microphone"], SENSOR_SID["bone_acc"]):
                    fmt = SENSOR_FORMATS[sid]
                    expected_size = struct.calcsize(fmt)
                    timestamp_s = time_us / 1e6

                    try:
                        if size == expected_size:
                            values = struct.unpack_from(fmt, mm, payload_start)
                            chunk_data[sid]['timestamps'].append(timestamp_s)
                            chunk_data[sid]['values'].append(values)
                        elif sid in (SENSOR_SID["ppg_right"], SENSOR_SID["ppg_left"]) and (size - 2) % expected_size == 0:
                            sample_count = (size - 2) // expected_size
                            delta = struct.unpack_from('<H', mm, payload_end - 2)[0] / 1e6
                            raw_values = np.frombuffer(mm[payload_start:payload_end - 2], dtype='<u4').copy().reshape(sample_count, 4)
                            chunk_data[sid]['timestamps'].extend(timestamp_s + np.arange(sample_count, dtype=np.float64) * delta)
                            chunk_data[sid]['values'].extend(raw_values.tolist())
                    except (struct.error, ValueError):
                        metadata['truncated'] = True
                        break

                pos = payload_end

    for sid, sensor_chunk in chunk_data.items():
        sensor_name = SID_SENSOR[sid]
        if sensor_chunk['timestamps']:
            sensor_chunk['timestamps'] = np.asarray(sensor_chunk['timestamps'], dtype=np.float64)
            sensor_chunk['values'] = np.asarray(sensor_chunk['values'])
        else:
            sensor_chunk['timestamps'] = np.empty(0, dtype=np.float64)
            sensor_chunk['values'] = np.empty((0, len(LABELS[sensor_name])))

    return chunk_data, metadata


def _choose_parse_worker_count(data_size: int) -> int:
    cpu_count = os.cpu_count() or 1
    if cpu_count <= 1 or data_size < PARSE_MIN_PARALLEL_FILE_SIZE:
        return 1
    return max(1, min(MAX_PARALLEL_PARSE_WORKERS, cpu_count, math.ceil(data_size / PARSE_TARGET_CHUNK_SIZE)))


def _emit_parse_progress(progress_callback: Optional[Callable[[int], None]],
                         processed_bytes: int,
                         total_bytes: int,
                         last_percent: int) -> int:
    if progress_callback is None or total_bytes <= 0:
        return last_percent
    percent = max(0, min(100, int((processed_bytes * 100) / total_bytes)))
    if percent != last_percent:
        progress_callback(percent)
    return percent


def _merge_chunk_results(chunk_results: list[tuple[dict, dict]]) -> tuple[dict, dict]:
    parsed = {}
    sensor_counts = {}
    total_samples = 0

    for sensor_name in ACTIVE_SENSORS:
        sid = SENSOR_SID[sensor_name]
        timestamps = []
        values = []
        for chunk_data, _ in chunk_results:
            sensor_chunk = chunk_data[sid]
            if sensor_chunk['timestamps'].size:
                timestamps.append(sensor_chunk['timestamps'])
                values.append(sensor_chunk['values'])

        if timestamps:
            parsed[sensor_name] = {
                'timestamps': np.concatenate(timestamps),
                'values': np.concatenate(values, axis=0),
            }
        else:
            parsed[sensor_name] = {
                'timestamps': np.empty(0, dtype=np.float64),
                'values': np.empty((0, len(LABELS[sensor_name]))),
            }

        sensor_counts[sensor_name] = int(parsed[sensor_name]['timestamps'].size)
        total_samples += sensor_counts[sensor_name]

    return parsed, {
        'truncated': any(chunk_metadata['truncated'] for _, chunk_metadata in chunk_results),
        'resync_count': sum(chunk_metadata['resync_count'] for _, chunk_metadata in chunk_results),
        'resync_positions': [pos for _, chunk_metadata in chunk_results for pos in chunk_metadata['resync_positions']],
        'sensor_counts': sensor_counts,
        'total_samples': total_samples,
    }


def build_dataframe_from_parsed(parsed: dict) -> pd.DataFrame:
    dfs = []
    for sensor_name in ACTIVE_SENSORS:
        sensor = parsed.get(sensor_name)
        if not sensor or sensor['timestamps'].size == 0:
            continue
        df = pd.DataFrame(sensor['values'], columns=LABELS[sensor_name], index=sensor['timestamps'])
        df.index.name = 'timestamp'
        df = df[~df.index.duplicated(keep='first')]
        dfs.append(df)

    if not dfs:
        return pd.DataFrame()

    common_index = pd.Index([])
    for df in dfs:
        common_index = common_index.union(df.index)
    common_index = common_index.sort_values()
    return pd.concat([df.reindex(common_index) for df in dfs], axis=1)


def parse_oe_file(filename: str,
                  progress_callback: Optional[Callable[[int], None]] = None) -> tuple[dict, dict]:
    metadata, data_offset, file_size = _parse_file_header(filename)
    requested_worker_count = _choose_parse_worker_count(file_size - data_offset)
    worker_count = requested_worker_count
    ranges = _build_parse_ranges(data_offset, file_size, worker_count)
    parse_args = [(filename, start, end, is_first, is_last, file_size) for start, end, is_first, is_last in ranges]
    total_bytes = max(1, file_size - data_offset)
    last_percent = _emit_parse_progress(progress_callback, 0, total_bytes, -1)

    if worker_count == 1:
        chunk_results = [_parse_oe_chunk(parse_args[0])]
        last_percent = _emit_parse_progress(progress_callback, total_bytes, total_bytes, last_percent)
    else:
        try:
            chunk_results = [None] * len(parse_args)
            processed_bytes = 0
            with ProcessPoolExecutor(max_workers=worker_count) as executor:
                future_to_index = {executor.submit(_parse_oe_chunk, args): index for index, args in enumerate(parse_args)}
                for future in as_completed(future_to_index):
                    index = future_to_index[future]
                    chunk_results[index] = future.result()
                    _, start, end, _, _, _ = parse_args[index]
                    processed_bytes += end - start
                    last_percent = _emit_parse_progress(progress_callback, processed_bytes, total_bytes, last_percent)
        except (PermissionError, OSError, NotImplementedError):
            worker_count = 1
            chunk_results = [_parse_oe_chunk((filename, data_offset, file_size, True, True, file_size))]
            last_percent = _emit_parse_progress(progress_callback, total_bytes, total_bytes, last_percent)

    parsed, merged_metadata = _merge_chunk_results(chunk_results)
    metadata.update(merged_metadata)
    metadata['parse_workers'] = worker_count
    metadata['parse_parallel_requested_workers'] = requested_worker_count
    _emit_parse_progress(progress_callback, total_bytes, total_bytes, last_percent)
    return parsed, metadata
