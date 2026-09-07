#!/usr/bin/env python3
"""Render closed/open RGB-D frames and verify markers plus drawer articulation."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import sys
import traceback
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from environment import build_environment


def load_rgb(path: Path) -> np.ndarray:
    image = np.asarray(Image.open(path).convert("RGB"))
    if image.shape != (480, 640, 3):
        raise AssertionError(f"expected 640x480 RGB, got {image.shape}")
    if float(image.std()) < 8 or int(image.max()) - int(image.min()) < 70:
        raise AssertionError("render is blank or nearly uniform")
    return image


def marker_centroid(image: np.ndarray, color: str) -> tuple[np.ndarray, int]:
    r = image[..., 0].astype(float)
    g = image[..., 1].astype(float)
    b = image[..., 2].astype(float)
    if color == "yellow":
        mask = (r > 145) & (g > 105) & (b < 100) & (r > 1.35 * b) & (g > 1.25 * b)
    elif color == "cyan":
        mask = (g > 130) & (b > 145) & (r < 105) & (g > 1.45 * r) & (b > 1.5 * r)
    elif color == "magenta":
        mask = (r > 140) & (b > 105) & (g < 100) & (r > 1.45 * g) & (b > 1.35 * g)
    else:
        raise ValueError(color)
    y, x = np.nonzero(mask)
    if len(x) < 20:
        raise AssertionError(f"{color} marker is not visible: pixels={len(x)}")
    return np.asarray([float(x.mean()), float(y.mean())]), int(len(x))


def run(args: argparse.Namespace) -> dict:
    backend = os.environ.get("MUJOCO_GL", "unset")
    if backend in {"unset", "disable"}:
        raise AssertionError("render smoke requires an enabled MUJOCO_GL backend")
    env = build_environment(args.model, args.spec)
    args.screenshot_dir.mkdir(parents=True, exist_ok=True)
    env.run_physics(100)
    before_capture = env.capture_rgbd(args.screenshot_dir / "before_capture")
    before_path = args.screenshot_dir / "before.png"
    shutil.copy2(before_capture["rgb"]["path"], before_path)

    env.reset()
    env.step({"id": "press_unlock_button", "payload": {"press_depth_m": 0.012}})
    env.step({"id": "pull_drawer_22cm", "payload": {"distance_m": 0.22}})
    inspected = env.step({
        "id": "inspect_open_drawer",
        "payload": {"output_dir": str(args.screenshot_dir / "after_capture")},
    })
    if not env.is_success():
        raise AssertionError("full unlock -> pull -> inspect task did not succeed")
    after_capture = inspected["sensor"]
    after_path = args.screenshot_dir / "after.png"
    shutil.copy2(after_capture["rgb"]["path"], after_path)

    before = load_rgb(before_path)
    after = load_rgb(after_path)
    marker_metrics = {}
    for color in ("yellow", "cyan", "magenta"):
        before_center, before_pixels = marker_centroid(before, color)
        after_center, after_pixels = marker_centroid(after, color)
        marker_metrics[color] = {
            "before_centroid_px": before_center.tolist(),
            "after_centroid_px": after_center.tolist(),
            "before_pixels": before_pixels,
            "after_pixels": after_pixels,
            "movement_px": float(np.linalg.norm(after_center - before_center)),
        }
    if marker_metrics["cyan"]["movement_px"] < 18:
        raise AssertionError(
            f"handle marker did not visibly articulate: {marker_metrics['cyan']['movement_px']:.2f}px"
        )

    before_depth = np.load(before_capture["depth"]["path"])
    after_depth = np.load(after_capture["depth"]["path"])
    if before_depth.shape != (480, 640) or after_depth.shape != (480, 640):
        raise AssertionError("unexpected depth dimensions")
    finite = np.isfinite(before_depth) & np.isfinite(after_depth)
    changed_depth_pixels = int((finite & (np.abs(before_depth - after_depth) > 0.002)).sum())
    if changed_depth_pixels < 300:
        raise AssertionError(f"too few depth pixels changed: {changed_depth_pixels}")

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "command": " ".join(sys.argv),
        "render_backend_requested": backend,
        "renderer_context": after_capture["renderer_context"],
        "resolution": [640, 480],
        "screenshots": {"before": str(before_path.resolve()), "after": str(after_path.resolve())},
        "visible_markers": marker_metrics,
        "drawer_articulation_m": inspected["drawer_joint_qpos"],
        "changed_depth_pixels": changed_depth_pixels,
        "task_success": True,
        "interaction_sequence": inspected["state"]["history"],
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--screenshot-dir", type=Path, default=base / "output" / "screenshots")
    parser.add_argument("--result", type=Path, default=base / "output" / "render_results.json")
    args = parser.parse_args()
    try:
        result, exit_code = run(args), 0
    except Exception as exc:
        result, exit_code = {
            "status": "FAIL",
            "mujoco_executed": True,
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "command": " ".join(sys.argv),
            "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }, 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
