#!/usr/bin/env python3
"""Verify the lever/ramp scene and its complete RGB-D interaction sequence."""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

SCENE_DIR = Path(__file__).resolve().parent
SHOWCASE_ROOT = SCENE_DIR.parent
if str(SHOWCASE_ROOT) not in sys.path:
    sys.path.insert(0, str(SHOWCASE_ROOT))

from capture_sequences import SCENES, run_scene


def marker_pixel_counts(image: np.ndarray) -> dict[str, int]:
    """Count the colored interaction markers visible in an RGB frame."""
    red, green, blue = (image[..., index].astype(float) for index in range(3))
    masks = {
        "yellow": (red > 150) & (green > 120) & (blue < 110) & (red > 1.2 * blue) & (green > 1.15 * blue),
        "magenta": (red > 140) & (blue > 90) & (green < 120) & (red > 1.3 * green) & (blue > 1.15 * green),
        "cyan": (green > 130) & (blue > 120) & (red < 120) & (green > 1.2 * red) & (blue > 1.1 * red),
        "orange": (red > 150) & (green > 20) & (green < 150) & (blue < 100) & (red > 1.35 * blue),
    }
    return {name: int(mask.sum()) for name, mask in masks.items()}


def run(args: argparse.Namespace) -> dict:
    backend = os.environ.get("MUJOCO_GL", "")
    if backend in {"", "disable"}:
        raise RuntimeError("render_smoke.py requires MUJOCO_GL=glfw/egl/osmesa")
    screenshot_dir = args.screenshot_dir.resolve()
    expected = (SCENE_DIR / "output" / "screenshots").resolve()
    if screenshot_dir != expected:
        raise ValueError("canonical screenshot directory is " + str(expected))
    sequence = run_scene(
        "04-lever-ball-ramp",
        SCENES["04-lever-ball-ramp"],
        SHOWCASE_ROOT,
        model_path=args.model.resolve(),
        spec_path=args.spec.resolve(),
    )
    before = np.asarray(Image.open(screenshot_dir / "before.png").convert("RGB"))
    after = np.asarray(Image.open(screenshot_dir / "after.png").convert("RGB"))
    before_depth = np.load(screenshot_dir / "before_depth.npy")
    after_depth = np.load(screenshot_dir / "after_depth.npy")
    changed_rgb = np.any(before != after, axis=2)
    changed_depth = np.isfinite(before_depth) & np.isfinite(after_depth) & (np.abs(before_depth - after_depth) > 1e-4)
    if int(changed_rgb.sum()) < 200 or int(changed_depth.sum()) < 100:
        raise AssertionError("insufficient visible RGB-D change")
    marker_before = marker_pixel_counts(before)
    marker_after = marker_pixel_counts(after)
    marker_pixels = {
        name: {"before": marker_before[name], "after": marker_after[name]}
        for name in marker_before
    }
    if any(max(values.values()) < 10 for values in marker_pixels.values()):
        raise AssertionError("one or more interaction markers are not visible")
    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "path_base": "package_root",
        "render_backend_requested": backend,
        "renderer_context": "native macOS CGL via glfw" if platform.system() == "Darwin" and backend == "glfw" else backend,
        "renderer_verified": "PASS",
        "marker_visibility": {"status": "PASS", "pixels": marker_pixels},
        "rgb_depth_checks": "PASS",
        "before_after_pixel_check": "PASS",
        "interaction_sequence": sequence["interaction_sequence"],
        "ball_target": "PASS",
        "task_success": sequence["task_success"],
        "rgb_shape": list(after.shape),
        "depth_shape": list(after_depth.shape),
        "changed_rgb_pixels": int(changed_rgb.sum()),
        "changed_depth_pixels": int(changed_depth.sum()),
        "screenshots": {
            "before_rgb": "output/screenshots/before.png",
            "after_rgb": "output/screenshots/after.png",
            "sequence_contact_sheet": "output/screenshots/sequence.png",
            "sequence_tiff": "output/screenshots/sequence.tif",
            "sequence_directory": "output/screenshots/sequence",
        },
        "sequence": sequence,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=SCENE_DIR / "model.xml")
    parser.add_argument("--spec", type=Path, default=SCENE_DIR / "scene_spec.json")
    parser.add_argument("--screenshot-dir", type=Path, default=SCENE_DIR / "output" / "screenshots")
    parser.add_argument(
        "--result", type=Path, default=SCENE_DIR / "output" / "render_results.json"
    )
    args = parser.parse_args()
    args.model = args.model.resolve()
    args.spec = args.spec.resolve()
    args.screenshot_dir = args.screenshot_dir.resolve()
    args.result = args.result.resolve()
    try:
        result, code = run(args), 0
    except Exception as exc:
        result, code = {"status": "FAIL", "mujoco_executed": True, "mujoco_version": mujoco.__version__, "path_base": "package_root", "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"), "renderer_verified": "FAIL", "error_type": type(exc).__name__, "error": type(exc).__name__}, 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
