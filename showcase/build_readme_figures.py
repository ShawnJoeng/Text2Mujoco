#!/usr/bin/env python3
"""Compose the README figures from committed showcase captures.

Both figures are derived from files the capture pipeline already produced, so
they stay consistent with the reports instead of becoming hand-made artwork:

* ``docs/showcase_gallery.png`` — the final verified frame of all eight scenes.
* ``docs/dense_filmstrip.png`` — evenly spaced pages of one dense TIFF, labelled
  with the simulation time recorded for that page in the dense report.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont

SHOWCASE = Path(__file__).resolve().parent
REPO = SHOWCASE.parent
DOCS = REPO / "docs"

GALLERY: Sequence[Tuple[str, str]] = (
    ("01-button-cube-box", "01  Button, Cube, Box"),
    ("02-smart-drawer", "02  Smart Tool Cabinet"),
    ("03-warehouse-navigation", "03  Warehouse Navigation"),
    ("04-lever-ball-ramp", "04  Lever and Ramp Ball"),
    ("05-robot-arm-sorting", "05  Arm Sorting Cell"),
    ("06-forklift-pallet", "06  Forklift Delivery"),
    ("07-robot-assembly", "07  Peg Assembly"),
    ("08-conveyor-arm", "08  Conveyor Handoff"),
)
FILMSTRIP_SCENE = "06-forklift-pallet"
FILMSTRIP_COLUMNS = 6

BACKGROUND = (14, 17, 22)
LABEL_FILL = (233, 237, 243)
SUBLABEL_FILL = (150, 160, 175)


def font(size: int) -> ImageFont.ImageFont:
    for candidate in (
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default()


def centered(draw: ImageDraw.ImageDraw, box: Tuple[int, int, int, int], text: str,
             type_face: ImageFont.ImageFont, fill: Tuple[int, int, int]) -> None:
    left, top, right, _ = box
    width = draw.textlength(text, font=type_face)
    draw.text((left + (right - left - width) / 2, top), text, font=type_face, fill=fill)


def build_gallery(output_path: Path, columns: int = 4, tile_width: int = 320) -> Dict[str, Any]:
    gap, label_h = 12, 28
    tile_height = tile_width * 3 // 4
    rows = (len(GALLERY) + columns - 1) // columns
    canvas = Image.new(
        "RGB",
        (columns * tile_width + (columns + 1) * gap,
         rows * (tile_height + label_h) + (rows + 1) * gap),
        BACKGROUND,
    )
    draw = ImageDraw.Draw(canvas)
    type_face = font(15)
    for index, (scene, label) in enumerate(GALLERY):
        source = SHOWCASE / scene / "output" / "screenshots" / "after.png"
        with Image.open(source) as frame:
            tile = frame.convert("RGB").resize((tile_width, tile_height), Image.LANCZOS)
        column, row = index % columns, index // columns
        x = gap + column * (tile_width + gap)
        y = gap + row * (tile_height + label_h + gap)
        canvas.paste(tile, (x, y))
        centered(draw, (x, y + tile_height + 7, x + tile_width, y + tile_height + label_h),
                 label, type_face, LABEL_FILL)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path)
    return {"figure": str(output_path.relative_to(REPO)), "tiles": len(GALLERY), "size": canvas.size}


def build_filmstrip(output_path: Path, scene: str = FILMSTRIP_SCENE,
                    columns: int = FILMSTRIP_COLUMNS, tile_width: int = 210) -> Dict[str, Any]:
    report = json.loads(
        (SHOWCASE / scene / "output" / "dense_sequence_results.json").read_text(encoding="utf-8")
    )
    records: List[Dict[str, Any]] = [record for record in report["frames"] if not record["event_frame"]]
    if len(records) < columns:
        raise SystemExit(f"{scene} has too few regular dense frames for a filmstrip")
    # Even spacing across the run keeps the strip readable while the labels stay
    # the real simulation times, so the 0.20 s cadence is verifiable by eye.
    picks = [records[round(i * (len(records) - 1) / (columns - 1))] for i in range(columns)]

    gap, label_h = 10, 30
    tile_height = tile_width * 3 // 4
    canvas = Image.new(
        "RGB",
        (columns * tile_width + (columns + 1) * gap, tile_height + label_h + 2 * gap),
        BACKGROUND,
    )
    draw = ImageDraw.Draw(canvas)
    time_face, page_face = font(15), font(12)
    with Image.open(SHOWCASE / scene / "output" / "screenshots" / "dense_sequence.tif") as tiff:
        for column, record in enumerate(picks):
            tiff.seek(int(record["archive_frame"]) - 1)
            tile = tiff.convert("RGB").resize((tile_width, tile_height), Image.LANCZOS)
            x = gap + column * (tile_width + gap)
            canvas.paste(tile, (x, gap))
            centered(draw, (x, gap + tile_height + 5, x + tile_width, gap + tile_height + 20),
                     f"t = {float(record['actual_time_s']):.2f} s", time_face, LABEL_FILL)
            centered(draw, (x, gap + tile_height + 21, x + tile_width, gap + tile_height + label_h),
                     f"TIFF page {int(record['archive_frame'])}", page_face, SUBLABEL_FILL)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path)
    return {
        "figure": str(output_path.relative_to(REPO)),
        "scene": scene,
        "pages": [int(record["archive_frame"]) for record in picks],
        "times_s": [round(float(record["actual_time_s"]), 3) for record in picks],
        "size": canvas.size,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--docs-dir", type=Path, default=DOCS)
    args = parser.parse_args()
    docs_dir = args.docs_dir.resolve()
    if REPO not in docs_dir.parents and docs_dir != REPO:
        raise SystemExit("figures must be written inside the repository")
    reports = [
        build_gallery(docs_dir / "showcase_gallery.png"),
        build_filmstrip(docs_dir / "dense_filmstrip.png"),
    ]
    print(json.dumps(reports, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
