#!/usr/bin/env python3
"""Analyze and replay STM32H743 IMU_LOG/*.IMU recordings.

The default path has no third-party dependencies.  Use --plot only when
matplotlib is installed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import struct
from pathlib import Path
from typing import Iterable


MAGIC = 0x4C554D49
VERSION = 1
HEADER_SIZE = 128
RECORD = struct.Struct("<II3h3h3h3hhH7f4iI")
GYRO_LSB_PER_DPS = 65.5
ACCEL_LSB_PER_G = 8192.0
GYRO_DEADBAND_DPS = 0.10


def unwrap_degrees(values: Iterable[float]) -> list[float]:
    result: list[float] = []
    previous = 0.0
    total = 0.0
    for index, value in enumerate(values):
        if index == 0:
            total = value
        else:
            delta = value - previous
            while delta > 180.0:
                delta -= 360.0
            while delta < -180.0:
                delta += 360.0
            total += delta
        previous = value
        result.append(total)
    return result


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else float("nan")


def std(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else float("nan")


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(v * v for v in values)) if values else float("nan")


def read_log(path: Path) -> tuple[dict, list[dict]]:
    data = path.read_bytes()
    if len(data) < HEADER_SIZE:
        raise ValueError("file is shorter than the 128-byte IMU header")

    magic, version, header_size, record_size, period_ms = struct.unpack_from(
        "<IHHHH", data, 0
    )
    if magic != MAGIC:
        raise ValueError(f"bad magic 0x{magic:08X}; expected 0x{MAGIC:08X}")
    if version != VERSION:
        raise ValueError(f"unsupported version {version}")
    if header_size != HEADER_SIZE or record_size != RECORD.size:
        raise ValueError(
            f"layout mismatch: header={header_size}, record={record_size}, "
            f"tool expects {HEADER_SIZE}/{RECORD.size}"
        )

    header = {
        "version": version,
        "period_ms": period_ms,
        "start_tick_ms": struct.unpack_from("<I", data, 12)[0],
        "calibration_sequence": struct.unpack_from("<I", data, 16)[0],
        "gyro_bias_raw": list(struct.unpack_from("<3f", data, 20)),
        "accel_offset_raw": list(struct.unpack_from("<3f", data, 32)),
        "gyro_std_dps": list(struct.unpack_from("<3f", data, 44)),
        "accel_std_g": list(struct.unpack_from("<3f", data, 56)),
        "calibration_temperature_c": struct.unpack_from("<f", data, 68)[0],
        "persistent_calibration": bool(data[72]),
    }

    payload = memoryview(data)[header_size:]
    complete_size = (len(payload) // record_size) * record_size
    records: list[dict] = []
    for offset in range(0, complete_size, record_size):
        values = RECORD.unpack_from(payload, offset)
        records.append(
            {
                "tick_ms": values[0],
                "sequence": values[1],
                "raw_ax": values[2],
                "raw_ay": values[3],
                "raw_az": values[4],
                "raw_gx": values[5],
                "raw_gy": values[6],
                "raw_gz": values[7],
                "ax": values[8],
                "ay": values[9],
                "az": values[10],
                "gx": values[11],
                "gy": values[12],
                "gz": values[13],
                "temperature_raw": values[14],
                "flags": values[15],
                "pitch_deg": values[16],
                "roll_deg": values[17],
                "yaw_deg": values[18],
                "yaw_rate_dps": values[19],
                "odom_x_mm": values[20],
                "odom_y_mm": values[21],
                "odom_heading_deg": values[22],
                "wheel_1": values[23],
                "wheel_2": values[24],
                "wheel_3": values[25],
                "wheel_4": values[26],
                "imu_failure_count": values[27],
            }
        )

    header["trailing_bytes"] = len(payload) - complete_size
    return header, records


def replay_planar_yaw(records: list[dict], gyro_bias_z: float) -> list[float]:
    if not records:
        return []
    mount_sign = -1.0 if records[0]["raw_az"] >= 0 else 1.0
    result = [0.0]
    previous_tick = records[0]["tick_ms"]
    previous_rate = 0.0
    yaw = 0.0
    for record in records[1:]:
        dt = ((record["tick_ms"] - previous_tick) & 0xFFFFFFFF) * 0.001
        previous_tick = record["tick_ms"]
        rate = mount_sign * ((record["raw_gz"] - gyro_bias_z) / GYRO_LSB_PER_DPS)
        if abs(rate) < GYRO_DEADBAND_DPS:
            rate = 0.0
        if 0.005 <= dt <= 0.030:
            yaw += 0.5 * (previous_rate + rate) * dt
        else:
            yaw += rate * 0.010
        previous_rate = rate
        while yaw > 180.0:
            yaw -= 360.0
        while yaw < -180.0:
            yaw += 360.0
        result.append(yaw)
    return result


def analyze(header: dict, records: list[dict]) -> tuple[dict, list[float]]:
    if len(records) < 2:
        raise ValueError("at least two complete IMU records are required")

    deltas_ms = [
        (records[i]["tick_ms"] - records[i - 1]["tick_ms"]) & 0xFFFFFFFF
        for i in range(1, len(records))
    ]
    duration_s = sum(deltas_ms) * 0.001
    replay_yaw = replay_planar_yaw(records, header["gyro_bias_raw"][2])
    firmware_yaw = unwrap_degrees(r["yaw_deg"] for r in records)
    replay_unwrapped = unwrap_degrees(replay_yaw)
    replay_error = [
        replay_unwrapped[i] - replay_unwrapped[0]
        - (firmware_yaw[i] - firmware_yaw[0])
        for i in range(len(records))
    ]

    stationary = []
    for record in records:
        accel_norm = math.sqrt(
            record["ax"] ** 2 + record["ay"] ** 2 + record["az"] ** 2
        ) / ACCEL_LSB_PER_G
        gyro_max = max(abs(record[a]) for a in ("gx", "gy", "gz")) / GYRO_LSB_PER_DPS
        if abs(accel_norm - 1.0) <= 0.06 and gyro_max <= 0.60:
            stationary.append(record)

    gyro_axes = ("gx", "gy", "gz")
    corrected_gyro_mean = [
        mean([r[axis] / GYRO_LSB_PER_DPS for r in stationary]) for axis in gyro_axes
    ]
    corrected_gyro_std = [
        std([r[axis] / GYRO_LSB_PER_DPS for r in stationary]) for axis in gyro_axes
    ]
    accel_norms = [
        math.sqrt(r["ax"] ** 2 + r["ay"] ** 2 + r["az"] ** 2)
        / ACCEL_LSB_PER_G
        for r in records
    ]
    yaw_change = firmware_yaw[-1] - firmware_yaw[0]
    summary = {
        "records": len(records),
        "duration_s": duration_s,
        "sample_rate_hz": (len(records) - 1) / duration_s if duration_s else None,
        "dt_ms_mean": mean([float(v) for v in deltas_ms]),
        "dt_ms_std": std([float(v) for v in deltas_ms]),
        "timing_gaps_over_30ms": sum(v > 30 for v in deltas_ms),
        "sequence_gaps": sum(
            ((records[i]["sequence"] - records[i - 1]["sequence"]) & 0xFFFFFFFF) != 1
            for i in range(1, len(records))
        ),
        "stationary_samples": len(stationary),
        "stationary_ratio": len(stationary) / len(records),
        "corrected_gyro_mean_dps": corrected_gyro_mean,
        "corrected_gyro_std_dps": corrected_gyro_std,
        "accel_norm_mean_g": mean(accel_norms),
        "accel_norm_std_g": std(accel_norms),
        "firmware_yaw_change_deg": yaw_change,
        "firmware_yaw_drift_deg_per_min": yaw_change / duration_s * 60.0 if duration_s else None,
        "replay_vs_firmware_yaw_rms_deg": rms(replay_error),
        "imu_read_failures_added": (
            records[-1]["imu_failure_count"] - records[0]["imu_failure_count"]
        ) & 0xFFFFFFFF,
        "final_odometry": {
            "x_mm": records[-1]["odom_x_mm"],
            "y_mm": records[-1]["odom_y_mm"],
            "heading_deg": records[-1]["odom_heading_deg"],
        },
        "calibration": header,
    }
    return summary, replay_yaw


def write_csv(path: Path, records: list[dict], replay_yaw: list[float]) -> None:
    fieldnames = list(records[0].keys()) + ["replay_yaw_deg"]
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record, yaw in zip(records, replay_yaw):
            row = dict(record)
            row["replay_yaw_deg"] = yaw
            writer.writerow(row)


def write_plot(path: Path, records: list[dict], replay_yaw: list[float]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("--plot requires matplotlib: pip install matplotlib") from exc

    start = records[0]["tick_ms"]
    time_s = [((r["tick_ms"] - start) & 0xFFFFFFFF) * 0.001 for r in records]
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(time_s, [r["pitch_deg"] for r in records], label="pitch")
    axes[0].plot(time_s, [r["roll_deg"] for r in records], label="roll")
    axes[0].legend()
    axes[0].set_ylabel("degree")
    axes[1].plot(time_s, [r["yaw_deg"] for r in records], label="firmware yaw")
    axes[1].plot(time_s, replay_yaw, label="replay yaw", alpha=0.75)
    axes[1].legend()
    axes[1].set_ylabel("degree")
    for axis, name in zip(("gx", "gy", "gz"), ("gx", "gy", "gz")):
        axes[2].plot(time_s, [r[axis] / GYRO_LSB_PER_DPS for r in records], label=name)
    axes[2].legend()
    axes[2].set_ylabel("degree/s")
    axes[2].set_xlabel("time (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=150)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="IMUxxxx.IMU file copied from SD")
    parser.add_argument("--csv", type=Path, help="write all decoded records as CSV")
    parser.add_argument("--json", type=Path, help="write analysis summary as JSON")
    parser.add_argument("--plot", type=Path, help="write a PNG plot (requires matplotlib)")
    args = parser.parse_args()

    header, records = read_log(args.log)
    summary, replay_yaw = analyze(header, records)
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=True))
    if args.csv:
        write_csv(args.csv, records, replay_yaw)
    if args.json:
        args.json.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=True),
            encoding="utf-8",
        )
    if args.plot:
        write_plot(args.plot, records, replay_yaw)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
