#!/usr/bin/env python3
"""Convert an MP4 (or any FFmpeg-decodable video) into a raw MJPEG stream.

Output format:
- raw concatenated JPEG frames
- file extension: .mjpeg

This matches the RAW path in the current STM32 MJPEG player.

Dependencies:
- ffmpeg available in PATH
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

LCD_WIDTH = 240
LCD_HEIGHT = 240
DEFAULT_FPS = 30
DEFAULT_QSCALE = 31
DEFAULT_PIX_FMT = "yuvj420p"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a video into a raw .mjpeg stream for STM32 playback."
    )
    parser.add_argument("input", nargs="?", type=Path, help="Input video file, e.g. xinyanyongran.mp4")
    parser.add_argument("output_dir", nargs="?", type=Path, help="Output folder or .mjpeg file path")
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
    parser.add_argument(
        "--qscale",
        type=int,
        default=DEFAULT_QSCALE,
        help=f"MJPEG quality scale, 2(best)-31(worst), default: {DEFAULT_QSCALE}",
    )
    parser.add_argument(
        "--pix-fmt",
        default=DEFAULT_PIX_FMT,
        choices=["yuvj420p", "yuvj422p", "yuvj444p", "gray"],
        help=f"MJPEG pixel format (default: {DEFAULT_PIX_FMT})",
    )
    parser.add_argument(
        "--stm32-safe",
        action="store_true",
        help=(
            "Use conservative STM32 preset: yuvj422p, fps capped to 20, "
            "qscale at least 5."
        ),
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
        args.output_dir = prompt_path("请输入输出MJPEG文件夹", args.input.parent)

    output_dir = args.output_dir
    if output_dir.suffix.lower() == ".mjpeg":
        output_path = output_dir
    else:
        output_path = output_dir / f"{args.input.stem}.mjpeg"

    return args.input, output_path


def build_ffmpeg_filter(width: int, height: int, fps: int, stretch: bool, background: str) -> str:
    if stretch:
        scale_part = f"scale={width}:{height}:flags=lanczos"
    else:
        scale_part = (
            f"scale={width}:{height}:force_original_aspect_ratio=decrease:flags=lanczos,"
            f"pad={width}:{height}:(ow-iw)/2:(oh-ih)/2:{background}"
        )

    return f"{scale_part},fps={fps},format={DEFAULT_PIX_FMT}"


def run_ffmpeg_to_mjpeg(args: argparse.Namespace) -> int:
    args.input, args.output = resolve_paths(args)

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

    if args.qscale < 2 or args.qscale > 31:
        print("qscale must be in [2, 31].", file=sys.stderr)
        return 1

    effective_fps = args.fps
    effective_qscale = args.qscale
    effective_pix_fmt = args.pix_fmt

    if args.stm32_safe:
        if effective_fps > 20:
            effective_fps = 20
        if effective_qscale < 5:
            effective_qscale = 5
        effective_pix_fmt = "yuvj422p"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    filter_expr = build_ffmpeg_filter(args.width, args.height, effective_fps, args.stretch, args.background)

    ffmpeg_cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        str(args.input),
        "-an",
        "-vf",
        filter_expr,
        "-c:v",
        "mjpeg",
        "-huffman",
        "default",
        "-q:v",
        str(effective_qscale),
        "-pix_fmt",
        effective_pix_fmt,
        "-f",
        "mjpeg",
        str(args.output),
    ]

    try:
        completed = subprocess.run(
            ffmpeg_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
    except FileNotFoundError:
        print("ffmpeg not found in PATH. Please install ffmpeg first.", file=sys.stderr)
        return 1

    if completed.returncode != 0:
        stderr_text = completed.stderr.decode(errors="replace").strip()
        print("FFmpeg failed.", file=sys.stderr)
        if stderr_text:
            print(stderr_text, file=sys.stderr)
        return completed.returncode

    if not args.output.is_file() or args.output.stat().st_size == 0:
        print("Output MJPEG was not generated correctly.", file=sys.stderr)
        return 1

    print(
        f"Wrote {args.output} -> raw MJPEG, {args.width}x{args.height}, "
        f"fps {effective_fps}, qscale {effective_qscale}, "
        f"pix_fmt {effective_pix_fmt}, size {args.output.stat().st_size} bytes"
    )

    if args.stm32_safe:
        print("Applied STM32-safe preset.")

    return 0


def main() -> int:
    return run_ffmpeg_to_mjpeg(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())