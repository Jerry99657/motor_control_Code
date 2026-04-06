#!/usr/bin/env python3
"""Analyze AVI/MJPEG files for STM32 playback compatibility.

The report focuses on issues that commonly cause STM32 HAL JPEG decode failures:
- missing SOI/EOI markers
- missing DHT tables
- progressive JPEG frames
- mixed/unstable dimensions or subsampling
- grayscale-heavy streams
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO


def read_u32_le(data: bytes) -> int:
    return struct.unpack("<I", data)[0]


def align2(value: int) -> int:
    return value + (value & 1)


def fourcc_to_text(fourcc: bytes) -> str:
    return "".join(chr(c) if 32 <= c <= 126 else "." for c in fourcc)


def is_video_chunk_id(chunk_id: bytes) -> bool:
    return (
        len(chunk_id) == 4
        and chunk_id[0:2].isdigit()
        and chunk_id[2:4].lower() == b"dc"
    )


@dataclass
class FrameCheckResult:
    has_soi: bool
    has_eoi: bool
    soi_offset: int
    has_dht: bool
    progressive: bool
    format_error: bool
    width: int
    height: int
    components: int
    subsampling: str
    colorspace: str


@dataclass
class Report:
    file_size: int = 0
    movi_start: int = 0
    movi_end: int = 0
    riff_ok: bool = False
    avi_ok: bool = False

    video_chunks_seen: int = 0
    video_chunks_checked: int = 0
    stream_counter: Counter[str] = field(default_factory=Counter)

    size_min: int = 0
    size_max: int = 0
    size_sum: int = 0

    soi_missing: int = 0
    eoi_missing: int = 0
    soi_offset_gt16: int = 0
    dht_missing: int = 0
    progressive_count: int = 0
    format_error_count: int = 0

    colorspace_counter: Counter[str] = field(default_factory=Counter)
    subsampling_counter: Counter[str] = field(default_factory=Counter)
    dimension_counter: Counter[str] = field(default_factory=Counter)

    sample_issues: list[str] = field(default_factory=list)


@dataclass
class ScanArgs:
    stream: str | None
    max_frames: int
    show_issues: int


def analyze_jpeg_payload(payload: bytes) -> FrameCheckResult:
    soi = payload.find(b"\xFF\xD8")
    has_soi = soi >= 0
    if not has_soi:
        return FrameCheckResult(
            has_soi=False,
            has_eoi=False,
            soi_offset=-1,
            has_dht=False,
            progressive=False,
            format_error=False,
            width=0,
            height=0,
            components=0,
            subsampling="unknown",
            colorspace="unknown",
        )

    eoi = payload.find(b"\xFF\xD9", soi + 2)
    has_eoi = eoi >= 0

    scan_end = (eoi + 2) if has_eoi else len(payload)
    has_dht = False
    progressive = False
    format_error = False
    width = 0
    height = 0
    components = 0
    h_samp = 0
    v_samp = 0

    pos = soi + 2
    while pos + 1 < scan_end:
        while pos < scan_end and payload[pos] != 0xFF:
            pos += 1
        if pos >= scan_end:
            break

        while pos < scan_end and payload[pos] == 0xFF:
            pos += 1
        if pos >= scan_end:
            break

        marker = payload[pos]
        pos += 1

        if marker == 0x00:
            continue
        if marker == 0xD9:
            break
        if marker == 0xDA:
            break
        if marker == 0x01 or (0xD0 <= marker <= 0xD7):
            continue

        if pos + 1 >= scan_end:
            format_error = True
            break

        seg_len = (payload[pos] << 8) | payload[pos + 1]
        if seg_len < 2 or (pos + seg_len) > scan_end:
            format_error = True
            break

        if marker == 0xC4:
            has_dht = True

        if marker in (0xC0, 0xC1, 0xC2):
            progressive = marker == 0xC2
            if seg_len >= 8:
                height = (payload[pos + 3] << 8) | payload[pos + 4]
                width = (payload[pos + 5] << 8) | payload[pos + 6]
                components = payload[pos + 7]
                if components >= 1 and seg_len >= 11:
                    hv = payload[pos + 9]
                    h_samp = (hv >> 4) & 0x0F
                    v_samp = hv & 0x0F

        pos += seg_len

    if components == 1:
        colorspace = "gray"
        subsampling = "gray"
    elif components == 3:
        colorspace = "ycbcr"
        if (h_samp, v_samp) == (2, 2):
            subsampling = "420"
        elif (h_samp, v_samp) == (2, 1):
            subsampling = "422"
        elif (h_samp, v_samp) == (1, 1):
            subsampling = "444"
        elif (h_samp, v_samp) == (0, 0):
            subsampling = "unknown"
        else:
            subsampling = f"{h_samp}x{v_samp}"
    elif components > 0:
        colorspace = f"components={components}"
        subsampling = "unknown"
    else:
        colorspace = "unknown"
        subsampling = "unknown"

    return FrameCheckResult(
        has_soi=has_soi,
        has_eoi=has_eoi,
        soi_offset=soi,
        has_dht=has_dht,
        progressive=progressive,
        format_error=format_error,
        width=width,
        height=height,
        components=components,
        subsampling=subsampling,
        colorspace=colorspace,
    )


def read_chunk_header(fp: BinaryIO, pos: int) -> tuple[bytes, int] | None:
    fp.seek(pos)
    hdr = fp.read(8)
    if len(hdr) != 8:
        return None
    return hdr[:4], read_u32_le(hdr[4:8])


def find_movi_region(fp: BinaryIO, file_size: int) -> tuple[bool, bool, int, int]:
    fp.seek(0)
    riff = fp.read(12)
    if len(riff) != 12:
        return False, False, 0, 0

    riff_ok = riff[0:4] == b"RIFF"
    avi_ok = riff[8:12] == b"AVI "
    if not (riff_ok and avi_ok):
        return riff_ok, avi_ok, 0, 0

    riff_size = read_u32_le(riff[4:8])
    riff_end = min(file_size, 8 + riff_size)

    pos = 12
    while pos + 8 <= riff_end:
        chunk = read_chunk_header(fp, pos)
        if chunk is None:
            break

        chunk_id, chunk_size = chunk
        data_pos = pos + 8
        next_pos = align2(data_pos + chunk_size)
        if next_pos <= pos or next_pos > riff_end:
            break

        if chunk_id == b"LIST" and chunk_size >= 4:
            fp.seek(data_pos)
            list_type = fp.read(4)
            if list_type == b"movi":
                return True, True, data_pos + 4, data_pos + chunk_size

        pos = next_pos

    return True, True, 0, 0


def scan_movi_region(
    fp: BinaryIO,
    start: int,
    end: int,
    report: Report,
    args: ScanArgs,
) -> None:
    pos = start

    while pos + 8 <= end:
        chunk = read_chunk_header(fp, pos)
        if chunk is None:
            return

        chunk_id, chunk_size = chunk
        data_pos = pos + 8
        next_pos = align2(data_pos + chunk_size)
        if next_pos <= pos or next_pos > end:
            return

        if chunk_id == b"LIST" and chunk_size >= 4:
            fp.seek(data_pos)
            list_type = fp.read(4)
            _ = list_type
            sub_start = data_pos + 4
            sub_end = data_pos + chunk_size
            if sub_end <= end and sub_start < sub_end:
                scan_movi_region(fp, sub_start, sub_end, report, args)
                if args.max_frames > 0 and report.video_chunks_checked >= args.max_frames:
                    return
            pos = next_pos
            continue

        if is_video_chunk_id(chunk_id):
            stream_id = chunk_id[0:2].decode("ascii", errors="replace")
            report.video_chunks_seen += 1
            report.stream_counter[stream_id] += 1

            stream_match = args.stream is None or stream_id == args.stream
            frame_limit_ok = args.max_frames == 0 or report.video_chunks_checked < args.max_frames

            if stream_match and frame_limit_ok:
                fp.seek(data_pos)
                payload = fp.read(chunk_size)
                report.video_chunks_checked += 1

                if report.video_chunks_checked == 1:
                    report.size_min = chunk_size
                    report.size_max = chunk_size
                else:
                    report.size_min = min(report.size_min, chunk_size)
                    report.size_max = max(report.size_max, chunk_size)
                report.size_sum += chunk_size

                result = analyze_jpeg_payload(payload)

                if not result.has_soi:
                    report.soi_missing += 1
                    if len(report.sample_issues) < args.show_issues:
                        report.sample_issues.append(
                            f"frame#{report.video_chunks_checked} stream={stream_id} missing SOI at 0x{data_pos:08X}"
                        )
                if result.has_soi and not result.has_eoi:
                    report.eoi_missing += 1
                    if len(report.sample_issues) < args.show_issues:
                        report.sample_issues.append(
                            f"frame#{report.video_chunks_checked} stream={stream_id} missing EOI at 0x{data_pos:08X}"
                        )
                if result.soi_offset > 16:
                    report.soi_offset_gt16 += 1
                if result.has_soi and not result.has_dht:
                    report.dht_missing += 1
                if result.progressive:
                    report.progressive_count += 1
                if result.format_error:
                    report.format_error_count += 1

                report.colorspace_counter[result.colorspace] += 1
                report.subsampling_counter[result.subsampling] += 1
                if result.width > 0 and result.height > 0:
                    report.dimension_counter[f"{result.width}x{result.height}"] += 1

            if args.max_frames > 0 and report.video_chunks_checked >= args.max_frames:
                return

        pos = next_pos


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze AVI/MJPEG compatibility for STM32 HAL JPEG playback."
    )
    parser.add_argument("input", nargs="?", type=Path, help="Input AVI path")
    parser.add_argument(
        "--stream",
        default=None,
        help="Optional stream id filter, e.g. 00 or 01",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Max frames to check (0 = all)",
    )
    parser.add_argument(
        "--show-issues",
        type=int,
        default=10,
        help="Max sample issue lines to print",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON report",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero if severe issues are found",
    )
    return parser.parse_args()


def prompt_input_path() -> Path:
    raw = input("请输入AVI文件地址: ").strip().strip('"')
    if not raw:
        raise ValueError("A value is required")
    return Path(raw)


def recommendations(report: Report) -> list[str]:
    recs: list[str] = []

    if report.video_chunks_seen == 0:
        recs.append("未发现任何 ??dc 视频块，AVI 封装可能不符合预期。")
        return recs

    if report.video_chunks_checked == 0:
        recs.append("没有命中可检查帧，请确认 --stream 参数是否正确。")
        return recs

    if report.progressive_count > 0:
        recs.append("检测到 progressive JPEG 帧；STM32 硬件解码通常需要 baseline。")

    if report.soi_missing > 0 or report.eoi_missing > 0:
        recs.append("存在 SOI/EOI 缺失帧，建议重新转码并检查源文件完整性。")

    dht_ratio = report.dht_missing / report.video_chunks_checked
    if dht_ratio > 0.30:
        recs.append("DHT 缺失比例较高，建议保留 DHT 或确保播放器端 DHT 注入逻辑稳定。")

    gray_ratio = report.colorspace_counter.get("gray", 0) / report.video_chunks_checked
    if gray_ratio > 0.30:
        recs.append("灰度帧比例较高，若源并非灰度视频，请强制转码为彩色 MJPEG。")

    if len(report.dimension_counter) > 1:
        recs.append("检测到多种分辨率，建议固定输出分辨率（如 240x240）。")

    if report.subsampling_counter.get("444", 0) > 0:
        recs.append("包含 4:4:4 帧，建议转为 4:2:2 或 4:2:0 以提高兼容性。")

    if not recs:
        recs.append("未发现明显封装异常，问题更可能在运行时链路（读取/缓存/时序）。")

    return recs


def severe_issue_count(report: Report) -> int:
    return (
        report.soi_missing
        + report.eoi_missing
        + report.progressive_count
        + report.format_error_count
    )


def print_human_report(report: Report, args: ScanArgs) -> None:
    print("=== AVI/MJPEG Check Report ===")
    print(f"File size: {report.file_size} bytes")
    print(f"RIFF header OK: {int(report.riff_ok)}")
    print(f"AVI signature OK: {int(report.avi_ok)}")

    if report.movi_start == 0 and report.movi_end == 0:
        print("movi region: not found")
        return

    print(f"movi region: 0x{report.movi_start:08X} - 0x{report.movi_end:08X}")
    print(f"Video chunks seen: {report.video_chunks_seen}")
    print(f"Video chunks checked: {report.video_chunks_checked}")

    if report.stream_counter:
        stream_text = ", ".join(
            f"{k}:{v}" for k, v in sorted(report.stream_counter.items(), key=lambda kv: kv[0])
        )
        print(f"Stream distribution: {stream_text}")

    if report.video_chunks_checked > 0:
        avg_size = report.size_sum / report.video_chunks_checked
        print(
            "Chunk size bytes: "
            f"min={report.size_min} max={report.size_max} avg={avg_size:.1f}"
        )

        print(
            "JPEG issues: "
            f"missing_soi={report.soi_missing} "
            f"missing_eoi={report.eoi_missing} "
            f"soi_offset_gt16={report.soi_offset_gt16} "
            f"missing_dht={report.dht_missing} "
            f"progressive={report.progressive_count} "
            f"format_err={report.format_error_count}"
        )

        if report.colorspace_counter:
            print(
                "Colorspace: "
                + ", ".join(
                    f"{k}:{v}" for k, v in report.colorspace_counter.most_common()
                )
            )

        if report.subsampling_counter:
            print(
                "Subsampling: "
                + ", ".join(
                    f"{k}:{v}" for k, v in report.subsampling_counter.most_common()
                )
            )

        if report.dimension_counter:
            print(
                "Dimensions: "
                + ", ".join(
                    f"{k}:{v}" for k, v in report.dimension_counter.most_common(5)
                )
            )

    if report.sample_issues:
        print("Sample issues:")
        for line in report.sample_issues:
            print(f"- {line}")

    print("Recommendations:")
    for line in recommendations(report):
        print(f"- {line}")

    if args.stream is not None:
        print(f"Checked stream filter: {args.stream}")
    if args.max_frames > 0:
        print(f"Frame check cap: {args.max_frames}")


def as_json_dict(report: Report) -> dict:
    return {
        "file_size": report.file_size,
        "riff_ok": report.riff_ok,
        "avi_ok": report.avi_ok,
        "movi_start": report.movi_start,
        "movi_end": report.movi_end,
        "video_chunks_seen": report.video_chunks_seen,
        "video_chunks_checked": report.video_chunks_checked,
        "stream_counter": dict(report.stream_counter),
        "chunk_size": {
            "min": report.size_min,
            "max": report.size_max,
            "sum": report.size_sum,
        },
        "issues": {
            "missing_soi": report.soi_missing,
            "missing_eoi": report.eoi_missing,
            "soi_offset_gt16": report.soi_offset_gt16,
            "missing_dht": report.dht_missing,
            "progressive": report.progressive_count,
            "format_error": report.format_error_count,
        },
        "colorspace_counter": dict(report.colorspace_counter),
        "subsampling_counter": dict(report.subsampling_counter),
        "dimension_counter": dict(report.dimension_counter),
        "sample_issues": report.sample_issues,
        "recommendations": recommendations(report),
    }


def main() -> int:
    cli = parse_args()

    if cli.input is None:
        try:
            cli.input = prompt_input_path()
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2

    input_path = cli.input
    if not input_path.is_file():
        print(f"Input file not found: {input_path}", file=sys.stderr)
        return 2

    stream = None
    if cli.stream is not None:
        stream = cli.stream.strip()
        if len(stream) != 2 or not stream.isdigit():
            print("--stream must be two digits like 00 or 01", file=sys.stderr)
            return 2

    args = ScanArgs(stream=stream, max_frames=max(0, int(cli.max_frames)), show_issues=max(0, int(cli.show_issues)))

    report = Report(file_size=input_path.stat().st_size)

    try:
        with input_path.open("rb") as fp:
            riff_ok, avi_ok, movi_start, movi_end = find_movi_region(fp, report.file_size)
            report.riff_ok = riff_ok
            report.avi_ok = avi_ok
            report.movi_start = movi_start
            report.movi_end = movi_end

            if not (riff_ok and avi_ok):
                if cli.json:
                    print(json.dumps(as_json_dict(report), ensure_ascii=False, indent=2))
                else:
                    print_human_report(report, args)
                return 1

            if movi_start > 0 and movi_end > movi_start:
                scan_movi_region(fp, movi_start, movi_end, report, args)

    except OSError as exc:
        print(f"Read failed: {exc}", file=sys.stderr)
        return 2

    if cli.json:
        print(json.dumps(as_json_dict(report), ensure_ascii=False, indent=2))
    else:
        print_human_report(report, args)

    if cli.strict and severe_issue_count(report) > 0:
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
