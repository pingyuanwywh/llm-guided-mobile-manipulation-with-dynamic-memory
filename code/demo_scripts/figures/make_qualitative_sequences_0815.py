#!/usr/bin/env python3
"""Build matching qualitative sequence figures for both 2026-08-15 runs."""

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


@dataclass(frozen=True)
class Experiment:
    key: str
    output_name: str
    run_dir: Path
    ceiling_video: str
    rgb_video: str
    depth_video: str
    ceiling_offset_s: float
    stages: tuple[Stage, ...]

    @property
    def output_dir(self) -> Path:
        return (
            DEMO_DIR
            / "edited"
            / self.run_dir.name
            / "figures"
            / f"qualitative_{self.output_name}"
        )


# Each non-initial timestamp is chosen during visual approach, when the object is
# complete and clear in the onboard views. Figure labels are relative to the first
# displayed frame (onboard t=8 s); onboard_time_s remains the source seek time.
# Ceiling offsets come from the embedded ceiling creation_time and the onboard
# recorder metadata wall_start_epoch.
EXPERIMENTS = {
    "gateB": Experiment(
        key="gateB",
        output_name="0815_gateB",
        run_dir=(
            DEMO_DIR
            / "raw"
            / "20260815_1858_gateB_2station-5can-SUCCESS-5of5"
        ),
        ceiling_video="WIN_20260815_18_57_27_Pro.mp4",
        rgb_video="20260815_185826_0815_gateB_rgb.mp4",
        depth_video="20260815_185826_0815_gateB_depth.mp4",
        ceiling_offset_s=58.4153671,
        stages=(
            Stage("t008", "(a) t=0 s", 8.0),
            Stage("t043", "(b) t=35 s", 43.0),
            Stage("t131", "(c) t=123 s", 131.0),
            Stage("t216", "(d) t=208 s", 216.0),
            Stage("t296", "(e) t=288 s", 296.0),
            Stage("t377", "(f) t=369 s", 377.0),
        ),
    ),
    "drone5": Experiment(
        key="drone5",
        output_name="0815_drone5",
        run_dir=(
            DEMO_DIR
            / "raw"
            / "20260815_2120_drone5_2station-5can-SUCCESS-5of5"
        ),
        ceiling_video="WIN_20260815_21_19_52_Pro.mp4",
        rgb_video="20260815_212045_0815_drone5_rgb.mp4",
        depth_video="20260815_212045_0815_drone5_depth.mp4",
        ceiling_offset_s=53.1038830,
        stages=(
            Stage("t008", "(a) t=0 s", 8.0),
            Stage("t038", "(b) t=30 s", 38.0),
            Stage("t119", "(c) t=111 s", 119.0),
            Stage("t197", "(d) t=189 s", 197.0),
            Stage("t274", "(e) t=266 s", 274.0),
            Stage("t355", "(f) t=347 s", 355.0),
        ),
    ),
}


def render(experiment: Experiment) -> tuple[Path, Path]:
    output_dir = experiment.output_dir
    frame_dir = output_dir / "source_frames"
    frame_dir.mkdir(parents=True, exist_ok=True)

    for stage in experiment.stages:
        extract_frame(
            experiment.run_dir / experiment.ceiling_video,
            stage.onboard_time_s + experiment.ceiling_offset_s,
            frame_dir / f"{stage.key}_ceiling.jpg",
        )
        extract_frame(
            experiment.run_dir / experiment.rgb_video,
            stage.onboard_time_s,
            frame_dir / f"{stage.key}_rgb.jpg",
        )
        extract_frame(
            experiment.run_dir / experiment.depth_video,
            stage.onboard_time_s,
            frame_dir / f"{stage.key}_depth.jpg",
        )

    page_width = PAGE_MARGIN * 2 + PANEL_WIDTH * 3 + GRID_GAP * 2
    page_height = PAGE_MARGIN * 2 + PANEL_HEIGHT * 2 + GRID_GAP
    page = Image.new("RGB", (page_width, page_height), "white")

    for index, stage in enumerate(experiment.stages):
        row, column = divmod(index, 3)
        x = PAGE_MARGIN + column * (PANEL_WIDTH + GRID_GAP)
        y = PAGE_MARGIN + row * (PANEL_HEIGHT + GRID_GAP)
        page.paste(build_panel(stage, frame_dir), (x, y))

    png_path = output_dir / f"qualitative_sequence_{experiment.output_name}.png"
    pdf_path = output_dir / f"qualitative_sequence_{experiment.output_name}.pdf"
    page.save(png_path, dpi=(300, 300), optimize=True)
    page.save(pdf_path, "PDF", resolution=300.0)
    return png_path, pdf_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "experiments",
        nargs="*",
        choices=tuple(EXPERIMENTS),
        help="Runs to render; defaults to both",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    selected = args.experiments or tuple(EXPERIMENTS)
    for key in selected:
        for output in render(EXPERIMENTS[key]):
            print(output)


if __name__ == "__main__":
    main()
