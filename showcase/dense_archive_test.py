#!/usr/bin/env python3
"""Read-only regression checks for committed dense GIF/TIFF archives."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parent
EXPECTED_INTERVAL_S = 0.20
EXPECTED_FRAME_MS = 200
EXPECTED_FINAL_MS = 800


def package_path(scene_dir: Path, value: str) -> Path:
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise AssertionError(f"{scene_dir.name}: archive path is not package-relative")
    resolved = (scene_dir / path).resolve()
    resolved.relative_to(scene_dir.resolve())
    return resolved


def verify_scene(scene_dir: Path) -> None:
    report_path = scene_dir / "output" / "dense_sequence_results.json"
    if not report_path.is_file():
        raise AssertionError(f"{scene_dir.name}: missing dense report")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("status") != "PASS":
        raise AssertionError(f"{scene_dir.name}: dense report is not PASS")
    if not math.isclose(float(report.get("simulation_interval_s", -1)), EXPECTED_INTERVAL_S, abs_tol=1e-9):
        raise AssertionError(f"{scene_dir.name}: unexpected simulation interval")
    frames = report.get("frames")
    if not isinstance(frames, list) or not frames:
        raise AssertionError(f"{scene_dir.name}: dense report has no frames")
    regular = [frame for frame in frames if not bool(frame.get("event_frame"))]
    if len(regular) < 2:
        raise AssertionError(f"{scene_dir.name}: dense report has too few regular frames")
    regular_times = [float(frame["actual_time_s"]) for frame in regular]
    if any(abs((right - left) - EXPECTED_INTERVAL_S) > 1e-6 for left, right in zip(regular_times, regular_times[1:])):
        raise AssertionError(f"{scene_dir.name}: regular frame timestamps are not 0.20 s apart")

    gif_path = package_path(scene_dir, str(report["gif"]))
    tiff_path = package_path(scene_dir, str(report["tiff"]))
    expected_count = len(frames)
    expected_durations = [EXPECTED_FRAME_MS] * expected_count
    expected_durations[-1] = EXPECTED_FINAL_MS
    with Image.open(gif_path) as gif:
        if int(getattr(gif, "n_frames", 1)) != expected_count:
            raise AssertionError(f"{scene_dir.name}: GIF frame count mismatch")
        durations = []
        for index in range(expected_count):
            gif.seek(index)
            durations.append(int(gif.info.get("duration", 0)))
            if float(np.asarray(gif.convert("RGB")).std()) < 5.0:
                raise AssertionError(f"{scene_dir.name}: GIF frame {index} is blank")
        if durations != expected_durations:
            raise AssertionError(f"{scene_dir.name}: GIF durations mismatch")
    with Image.open(tiff_path) as tiff:
        if int(getattr(tiff, "n_frames", 1)) != expected_count:
            raise AssertionError(f"{scene_dir.name}: TIFF page count mismatch")
        for index in range(expected_count):
            tiff.seek(index)
            if float(np.asarray(tiff.convert("RGB")).std()) < 5.0:
                raise AssertionError(f"{scene_dir.name}: TIFF page {index} is blank")


def main() -> int:
    scenes = sorted(path for path in ROOT.glob("0*") if path.is_dir())
    if not scenes:
        raise SystemExit("no showcase scenes found")
    for scene_dir in scenes:
        verify_scene(scene_dir)
        print(f"{scene_dir.name}: PASS")
    print("dense archive validation: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
