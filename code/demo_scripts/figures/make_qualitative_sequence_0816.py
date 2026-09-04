#!/usr/bin/env python3
"""Build a six-timestamp qualitative sequence figure for the 2026-08-16 run."""

from __future__ import annotations

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


RUN_DIR = Path(
    "~/jetrover_demo/raw/"
    "20260816_1207_drone5b_2station-5can-SUCCESS-5of5"
).expanduser()
CEILING_VIDEO = RUN_DIR / "WIN_20260816_12_06_33_Pro.mp4"
RGB_VIDEO = RUN_DIR / "20260816_120721_0816_drone5b_rgb.mp4"
DEPTH_VIDEO = RUN_DIR / "20260816_120721_0816_drone5b_depth.mp4"

# Embedded ceiling creation time is 12:06:33.000. The onboard recorder metadata
# reports 12:07:21.332, so onboard time 0 corresponds to ceiling time 48.332.
CEILING_OFFSET_S = 48.3322425

DEFAULT_OUTPUT_DIR = Path(
    "~/jetrover_demo/edited/"
    "20260816_1207_drone5b_2station-5can-SUCCESS-5of5/figures/"
    "qualitative_0816_drone5b"
).expanduser()
FONT_REGULAR = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
FONT_BOLD = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")


@dataclass(frozen=True)
class Stage:
    key: str
    label: str
    onboard_time_s: float


# Figure times are relative to the first displayed frame (onboard t=8 s),
# while onboard_time_s retains the absolute source-video seek time.
STAGES = (
    Stage("t008", "(a) t=0 s", 8.0),
    Stage("t037", "(b) t=29 s", 37.0),
    Stage("t141", "(c) t=133 s", 141.0),
    Stage("t232", "(d) t=224 s", 232.0),
    Stage("t329", "(e) t=321 s", 329.0),
    Stage("t438", "(f) t=430 s", 438.0),
)


PANEL_WIDTH = 900
PANEL_HEIGHT = 400
HEADER_HEIGHT = 44
CONTENT_HEIGHT = PANEL_HEIGHT - HEADER_HEIGHT
CEILING_WIDTH = 600
SIDE_WIDTH = PANEL_WIDTH - CEILING_WIDTH
SIDE_HEIGHT = CONTENT_HEIGHT // 2
GRID_GAP = 18
PAGE_MARGIN = 24


def extract_frame(video: Path, time_s: float, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    command = (
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-ss",
        f"{time_s:.3f}",
        "-i",
        str(video),
        "-frames:v",
        "1",
        "-q:v",
        "2",
        "-y",
        str(output),
    )
    subprocess.run(command, check=True)


def cover(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    target_width, target_height = size
    source_width, source_height = image.size
    scale = max(target_width / source_width, target_height / source_height)
    resized = image.resize(
        (round(source_width * scale), round(source_height * scale)),
        Image.Resampling.LANCZOS,
    )
    left = (resized.width - target_width) // 2
    top = (resized.height - target_height) // 2
    return resized.crop((left, top, left + target_width, top + target_height))


def add_view_tag(image: Image.Image, label: str, font: ImageFont.FreeTypeFont) -> None:
    draw = ImageDraw.Draw(image, "RGBA")
    box = draw.textbbox((0, 0), label, font=font)
    width = box[2] - box[0]
    height = box[3] - box[1]
    padding_x = 8
    padding_y = 5
    draw.rectangle(
        (8, 8, 8 + width + 2 * padding_x, 8 + height + 2 * padding_y),
        fill=(0, 0, 0, 168),
    )
    draw.text(
        (8 + padding_x, 8 + padding_y - box[1]),
        label,
        font=font,
        fill=(255, 255, 255, 255),
    )


def load_frame(path: Path, size: tuple[int, int], label: str) -> Image.Image:
    with Image.open(path) as source:
        image = cover(source.convert("RGB"), size)
    add_view_tag(image, label, ImageFont.truetype(str(FONT_REGULAR), 18))
    return image


def build_panel(
    stage: Stage,
    frame_dir: Path,
    primary_label: str = "Ceiling",
    primary_suffix: str = "ceiling",
) -> Image.Image:
    panel = Image.new("RGB", (PANEL_WIDTH, PANEL_HEIGHT), "white")
    draw = ImageDraw.Draw(panel)
    header_font = ImageFont.truetype(str(FONT_BOLD), 24)

    draw.text((14, 7), stage.label, font=header_font, fill=(25, 29, 33))
    draw.line(
        (0, HEADER_HEIGHT - 1, PANEL_WIDTH, HEADER_HEIGHT - 1),
        fill=(25, 29, 33),
        width=2,
    )

    ceiling = load_frame(
        frame_dir / f"{stage.key}_{primary_suffix}.jpg",
        (CEILING_WIDTH, CONTENT_HEIGHT),
        primary_label,
    )
    rgb = load_frame(
        frame_dir / f"{stage.key}_rgb.jpg",
        (SIDE_WIDTH, SIDE_HEIGHT),
        "Onboard RGB",
    )
    depth = load_frame(
        frame_dir / f"{stage.key}_depth.jpg",
        (SIDE_WIDTH, CONTENT_HEIGHT - SIDE_HEIGHT),
        "Depth",
    )

    panel.paste(ceiling, (0, HEADER_HEIGHT))
    panel.paste(rgb, (CEILING_WIDTH, HEADER_HEIGHT))
    panel.paste(depth, (CEILING_WIDTH, HEADER_HEIGHT + SIDE_HEIGHT))

    draw = ImageDraw.Draw(panel)
    draw.line(
        (CEILING_WIDTH, HEADER_HEIGHT, CEILING_WIDTH, PANEL_HEIGHT),
        fill="white",
        width=3,
    )
    draw.line(
        (
            CEILING_WIDTH,
            HEADER_HEIGHT + SIDE_HEIGHT,
            PANEL_WIDTH,
            HEADER_HEIGHT + SIDE_HEIGHT,
        ),
        fill="white",
        width=3,
    )
    draw.rectangle((0, 0, PANEL_WIDTH - 1, PANEL_HEIGHT - 1), outline=(25, 29, 33), width=2)
    return panel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for extracted frames and final figures",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    frame_dir = output_dir / "source_frames"
    frame_dir.mkdir(parents=True, exist_ok=True)

    for stage in STAGES:
        extract_frame(
            CEILING_VIDEO,
            stage.onboard_time_s + CEILING_OFFSET_S,
            frame_dir / f"{stage.key}_ceiling.jpg",
        )
        extract_frame(
            RGB_VIDEO,
            stage.onboard_time_s,
            frame_dir / f"{stage.key}_rgb.jpg",
        )
        extract_frame(
            DEPTH_VIDEO,
            stage.onboard_time_s,
            frame_dir / f"{stage.key}_depth.jpg",
        )

    page_width = PAGE_MARGIN * 2 + PANEL_WIDTH * 3 + GRID_GAP * 2
    page_height = PAGE_MARGIN * 2 + PANEL_HEIGHT * 2 + GRID_GAP
    page = Image.new("RGB", (page_width, page_height), "white")

    for index, stage in enumerate(STAGES):
        row, column = divmod(index, 3)
        x = PAGE_MARGIN + column * (PANEL_WIDTH + GRID_GAP)
        y = PAGE_MARGIN + row * (PANEL_HEIGHT + GRID_GAP)
        page.paste(build_panel(stage, frame_dir), (x, y))

    png_path = output_dir / "qualitative_sequence_0816_drone5b.png"
    pdf_path = output_dir / "qualitative_sequence_0816_drone5b.pdf"
    page.save(png_path, dpi=(300, 300), optimize=True)
    page.save(pdf_path, "PDF", resolution=300.0)
    print(png_path)
    print(pdf_path)


if __name__ == "__main__":
    main()
