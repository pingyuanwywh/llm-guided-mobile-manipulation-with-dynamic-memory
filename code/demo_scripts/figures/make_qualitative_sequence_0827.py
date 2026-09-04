#!/usr/bin/env python3
"""Build original-timeline or frame-dropped figures for the 2026-08-27 run."""

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
    / "20260827_2030_drone5_2station-5can-SUCCESS-5of5"
)
RGB_VIDEO = RUN_DIR / "20260827_203017_0827d_drone5_rgb.mp4"
DEPTH_VIDEO = RUN_DIR / "20260827_203017_0827d_drone5_depth.mp4"
EXTERNAL_VIDEOS = {
    1: RUN_DIR / "demo_night(1).mp4",
    2: RUN_DIR / "demo_night(2).mp4",
}
FRAME_DROPPED_DIR = (
    DEMO_DIR
    / "edited"
    / RUN_DIR.name
    / "intermediate"
    / "frame_dropped"
)
FRAME_DROPPED_RGB_VIDEO = (
    FRAME_DROPPED_DIR
    / "20260827_203017_0827d_drone5_rgb_frame_dropped.mp4"
)
FRAME_DROPPED_DEPTH_VIDEO = (
    FRAME_DROPPED_DIR
    / "20260827_203017_0827d_drone5_depth_frame_dropped.mp4"
)
FRAME_DROPPED_EXTERNAL_VIDEOS = {
    1: FRAME_DROPPED_DIR / "demo_night(1)_frame_dropped.mp4",
    2: FRAME_DROPPED_DIR / "demo_night(2)_frame_dropped.mp4",
}
DEFAULT_OUTPUT_DIR = (
    DEMO_DIR
    / "edited"
    / RUN_DIR.name
    / "figures"
    / "qualitative_0827_drone5"
)


@dataclass(frozen=True)
class SelectedStage:
    stage: Stage
    external_camera: int
    external_time_s: float


# Times are selected during the clear, successful visual-approach sequence for
# each can. In particular, can1 uses the second attempt, which grasped
# successfully. Figure labels are relative to the first displayed frame at
# onboard t=8 s. The external times use the visually estimated +29 s offset
# documented in qualitative_sync_timeline_0827_drone5.yaml.
SELECTED_STAGES = (
    SelectedStage(Stage("t008", "(a) t=0 s", 8.0), 1, 37.0),
    SelectedStage(Stage("t046", "(b) t=38 s", 46.0), 1, 75.0),
    SelectedStage(Stage("t139", "(c) t=131 s", 139.0), 1, 168.0),
    SelectedStage(Stage("t237", "(d) t=229 s", 237.0), 1, 266.0),
    SelectedStage(Stage("t372", "(e) t=364 s", 372.0), 2, 401.0),
    SelectedStage(Stage("t536", "(f) t=528 s", 536.0), 2, 565.0),
)

# These seek times address the derived videos after removing the 75 s failed
# can1 task. The final source frames are identical to the original-timeline
# selection, but their edited seek times and displayed time are 75 s earlier.
FRAME_DROPPED_SELECTED_STAGES = (
    *SELECTED_STAGES[:-1],
    SelectedStage(Stage("t461", "(f) t=453 s", 461.0), 2, 490.0),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for extracted frames and final figures",
    )
    parser.add_argument(
        "--frame-dropped",
        action="store_true",
        help="Extract from marked copies with the failed can1 task removed",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    if args.frame_dropped:
        selected_stages = FRAME_DROPPED_SELECTED_STAGES
        rgb_video = FRAME_DROPPED_RGB_VIDEO
        depth_video = FRAME_DROPPED_DEPTH_VIDEO
        external_videos = FRAME_DROPPED_EXTERNAL_VIDEOS
        frame_dir = output_dir / "source_frames_frame_dropped"
        output_suffix = "_frame_dropped"
    else:
        selected_stages = SELECTED_STAGES
        rgb_video = RGB_VIDEO
        depth_video = DEPTH_VIDEO
        external_videos = EXTERNAL_VIDEOS
        frame_dir = output_dir / "source_frames"
        output_suffix = ""
    frame_dir.mkdir(parents=True, exist_ok=True)

    for selected in selected_stages:
        stage = selected.stage
        extract_frame(
            external_videos[selected.external_camera],
            selected.external_time_s,
            frame_dir / f"{stage.key}_external.jpg",
        )
        extract_frame(
            rgb_video,
            stage.onboard_time_s,
            frame_dir / f"{stage.key}_rgb.jpg",
        )
        extract_frame(
            depth_video,
            stage.onboard_time_s,
            frame_dir / f"{stage.key}_depth.jpg",
        )

    page_width = PAGE_MARGIN * 2 + PANEL_WIDTH * 3 + GRID_GAP * 2
    page_height = PAGE_MARGIN * 2 + PANEL_HEIGHT * 2 + GRID_GAP
    page = Image.new("RGB", (page_width, page_height), "white")

    for index, selected in enumerate(selected_stages):
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

    png_path = output_dir / f"qualitative_sequence_0827_drone5{output_suffix}.png"
    pdf_path = output_dir / f"qualitative_sequence_0827_drone5{output_suffix}.pdf"
    page.save(png_path, dpi=(300, 300), optimize=True)
    page.save(pdf_path, "PDF", resolution=300.0)
    print(png_path)
    print(pdf_path)


if __name__ == "__main__":
    main()
