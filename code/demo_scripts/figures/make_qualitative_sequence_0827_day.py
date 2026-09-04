#!/usr/bin/env python3
"""Build the six-timestamp qualitative figure for the 2026-08-27 daytime run."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from PIL import Image

from make_qualitative_sequence_0816 import (
    GRID_GAP,
    PAGE_MARGIN,
    PANEL_HEIGHT,
    PANEL_WIDTH,
    Stage,
    build_panel,
    extract_frame,
)


DEMO_DIR = Path("~/jetrover_demo").expanduser()
RUN_DIR = (
    DEMO_DIR
    / "raw"
    / "20260827_1631_drone5_2station-5can-SUCCESS-5of5"
)
RGB_VIDEO = RUN_DIR / "20260827_163103_0827b_drone5_rgb.mp4"
DEPTH_VIDEO = RUN_DIR / "20260827_163103_0827b_drone5_depth.mp4"
EXTERNAL_VIDEOS = {
    "img": RUN_DIR / "IMG_3319.MOV",
    "vid": RUN_DIR / "VID_20260827_162913.mp4",
}
DEFAULT_OUTPUT_DIR = (
    DEMO_DIR
    / "edited"
    / RUN_DIR.name
    / "figures"
    / "qualitative_0827_day_drone5"
)


@dataclass(frozen=True)
class SelectedStage:
    stage: Stage
    external_camera: str
    external_time_s: float


# All frames come directly from the original videos. Figure times are relative
# to the initial displayed frame at onboard t=8 s. Each later timestamp shows a
# clear, successful approach to can5, can4, can3, can2, and can1 respectively.
SELECTED_STAGES = (
    SelectedStage(Stage("t008", "(a) t=0 s", 8.0), "vid", 118.425),
    SelectedStage(Stage("t039", "(b) t=31 s", 39.0), "vid", 149.425),
    SelectedStage(Stage("t135", "(c) t=127 s", 135.0), "img", 261.425),
    SelectedStage(Stage("t429", "(d) t=421 s", 429.0), "img", 555.425),
    SelectedStage(Stage("t560", "(e) t=552 s", 560.0), "img", 686.425),
    SelectedStage(Stage("t650", "(f) t=642 s", 650.0), "img", 776.425),
)


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

    for selected in SELECTED_STAGES:
        stage = selected.stage
        extract_frame(
            EXTERNAL_VIDEOS[selected.external_camera],
            selected.external_time_s,
            frame_dir / f"{stage.key}_external.jpg",
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

    for index, selected in enumerate(SELECTED_STAGES):
        row, column = divmod(index, 3)
        x = PAGE_MARGIN + column * (PANEL_WIDTH + GRID_GAP)
        y = PAGE_MARGIN + row * (PANEL_HEIGHT + GRID_GAP)
        panel = build_panel(
            selected.stage,
            frame_dir,
            primary_label="External View",
            primary_suffix="external",
        )
        page.paste(panel, (x, y))

    png_path = output_dir / "qualitative_sequence_0827_day_drone5.png"
    pdf_path = output_dir / "qualitative_sequence_0827_day_drone5.pdf"
    page.save(png_path, dpi=(300, 300), optimize=True)
    page.save(pdf_path, "PDF", resolution=300.0)
    print(png_path)
    print(pdf_path)


if __name__ == "__main__":
    main()
