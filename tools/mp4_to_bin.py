#!/usr/bin/env python3
"""Convert an MP4 (or any FFmpeg-decodable video) into the project's BIN animation format.

Output format:
- 32-byte little-endian header
- raw RGB565LE frame payload

The resulting .bin can be played by the current SD BIN browser/player.

Dependencies:
- ffmpeg available in PATH
"""

from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
from pathlib import Path

LCD_WIDTH = 240
LCD_HEIGHT = 240
MAGIC = 0x314E4151  # 'QAN1' in little endian storage
VERSION = 1
HEADER_SIZE = 32
DEFAULT_FPS = 10
DEFAULT_DELAY_MS = 1000 // DEFAULT_FPS


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a video into the STM32 BIN animation format."
    )
    parser.add_argument("input", nargs="?", type=Path, help="Input video file, e.g. xinyanyongran.mp4")
    parser.add_argument("output_dir", nargs="?", type=Path, help="Output folder for the .bin file")
    parser.add_argument("--width", type=int, default=LCD_WIDTH, help="Target width in pixels (default: 240)")
    parser.add_argument("--height", type=int, default=LCD_HEIGHT, help="Target height in pixels (default: 240)")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help=f"Output FPS (default: {DEFAULT_FPS})")
    parser.add_argument(
        "--stretch",
        action="store_true",
        help="Stretch to exactly width/height instead of keeping aspect ratio with black padding.",
    )
    parser.add_argument(
        "--background",
        default="black",
        help="Padding color when keep-aspect is used (default: black).",
    )
    return parser.parse_args()


def prompt_path(prompt_text: str, default_value: Path | None = None) -> Path:
    if default_value is not None:
        prompt_text = f"{prompt_text} [{default_value}]: "
    else:
        prompt_text = f"{prompt_text}: "

    raw_value = input(prompt_text).strip().strip('"')
    if raw_value == "":
        if default_value is None:
            raise ValueError("A value is required.")
        return default_value

    return Path(raw_value)


def resolve_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    if args.input is None:
        args.input = prompt_path("请输入MP4文件地址")

    if args.output_dir is None:
        args.output_dir = prompt_path("请输入输出BIN文件夹", args.input.parent)

    output_dir = args.output_dir
    if output_dir.suffix.lower() == ".bin":
        output_path = output_dir
        output_dir = output_path.parent
    else:
        output_path = output_dir / f"{args.input.stem}.bin"

    return args.input, output_path


def build_ffmpeg_filter(width: int, height: int, fps: int, stretch: bool, background: str) -> str:
    if stretch:
        scale_part = f"scale={width}:{height}:flags=lanczos"
    else:
        scale_part = (
            f"scale={width}:{height}:force_original_aspect_ratio=decrease:flags=lanczos,"
            f"pad={width}:{height}:(ow-iw)/2:(oh-ih)/2:{background}"
        )

    return f"{scale_part},fps={fps},format=rgb565le"


def run_ffmpeg_to_bin(args: argparse.Namespace) -> int:
    args.input, args.output = resolve_paths(args)

    def cleanup_output() -> None:
        if args.output.exists():
            try:
                args.output.unlink()
            except OSError:
                pass

    if not args.input.is_file():
        print(f"Input file not found: {args.input}", file=sys.stderr)
        return 1

    if args.width <= 0 or args.height <= 0:
        print("Width and height must be positive.", file=sys.stderr)
        return 1

    if args.fps <= 0:
        print("FPS must be positive.", file=sys.stderr)
        return 1

    if args.fps > 65535:
        print("FPS is too large.", file=sys.stderr)
        return 1

    frame_delay_ms = max(1, int(round(1000.0 / float(args.fps))))
    frame_size_bytes = args.width * args.height * 2
    filter_expr = build_ffmpeg_filter(args.width, args.height, args.fps, args.stretch, args.background)

    ffmpeg_cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(args.input),
        "-an",
        "-vf",
        filter_expr,
        "-pix_fmt",
        "rgb565le",
        "-f",
        "rawvideo",
        "pipe:1",
    ]

    args.output.parent.mkdir(parents=True, exist_ok=True)

    frame_count = 0
    payload_bytes = 0

    with open(args.output, "wb") as out_file:
        out_file.write(
            struct.pack(
                "<IHHHHHHIIII",
                MAGIC,
                VERSION,
                HEADER_SIZE,
                args.width,
                args.height,
                0,  # placeholder frame count
                frame_delay_ms,
                frame_size_bytes,
                0,  # placeholder payload size
                HEADER_SIZE,
                0,  # reserved to make the header 32 bytes
            )
        )

        process = subprocess.Popen(
            ffmpeg_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        try:
            pending = bytearray()
            while True:
                chunk = process.stdout.read(65536)
                if not chunk:
                    break

                pending.extend(chunk)
                while len(pending) >= frame_size_bytes:
                    out_file.write(pending[:frame_size_bytes])
                    del pending[:frame_size_bytes]
                    frame_count += 1
                    payload_bytes += frame_size_bytes

            if pending:
                raise RuntimeError(
                    f"FFmpeg output ended on a partial frame: got {len(pending)} trailing bytes, expected multiples of {frame_size_bytes}."
                )

            stderr_data = process.stderr.read()
            return_code = process.wait()
        except Exception:
            process.kill()
            process.wait()
            cleanup_output()
            raise

    if return_code != 0:
        stderr_text = stderr_data.decode(errors="replace").strip()
        cleanup_output()
        print("FFmpeg failed.", file=sys.stderr)
        if stderr_text:
            print(stderr_text, file=sys.stderr)
        return return_code

    if frame_count == 0:
        cleanup_output()
        print("No frames were produced. Check the input video and FFmpeg filter settings.", file=sys.stderr)
        return 1

    if frame_count > 0xFFFF:
        cleanup_output()
        print("Frame count exceeds the BIN format limit (65535).", file=sys.stderr)
        return 1

    payload_size = payload_bytes
    if payload_size > 0xFFFFFFFF:
        cleanup_output()
        print("Payload size exceeds the BIN format limit (4 GiB).", file=sys.stderr)
        return 1

    with open(args.output, "r+b") as out_file:
        out_file.seek(0)
        out_file.write(
            struct.pack(
                "<IHHHHHHIIII",
                MAGIC,
                VERSION,
                HEADER_SIZE,
                args.width,
                args.height,
                frame_count,
                frame_delay_ms,
                frame_size_bytes,
                payload_size,
                HEADER_SIZE,
                0,
            )
        )

    print(
        f"Wrote {args.output} -> {frame_count} frames, {args.width}x{args.height}, "
        f"delay {frame_delay_ms} ms, payload {payload_size} bytes"
    )
    return 0


def main() -> int:
    return run_ffmpeg_to_bin(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
