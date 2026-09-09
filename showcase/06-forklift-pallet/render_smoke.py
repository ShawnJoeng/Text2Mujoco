#!/usr/bin/env python3
"""Offscreen RGB-D and visible-marker smoke test for forklift delivery."""

from __future__ import annotations

import argparse
import json
import os
import platform
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from PIL import Image

from environment import build_environment


def load_rgb(path: Path, width: int, height: int) -> np.ndarray:
    image = np.asarray(Image.open(path).convert("RGB"))
    if image.shape != (height, width, 3):
        raise AssertionError(f"unexpected RGB shape: {image.shape}")
    if float(image.std()) < 5.0 or int(image.max()) - int(image.min()) < 50:
        raise AssertionError("RGB frame is blank or nearly uniform")
    return image


def color_pixels(image: np.ndarray, color: str, minimum: int = 12) -> int:
    r, g, b = (image[..., index].astype(float) for index in range(3))
    masks = {
        "yellow": (r > 130) & (g > 105) & (r > 1.25 * b) & (g > 1.45 * b),
        "cyan": (g > 95) & (b > 95) & (g > 1.30 * r) & (b > 1.30 * r),
        "green": (g > 100) & (g > 1.45 * r) & (g > 1.45 * b),
        "magenta": (r > 100) & (b > 70) & (r > 1.35 * g) & (b > 1.15 * g),
        "orange": (r > 120) & (r > 1.35 * g) & (g > 25) & (g > 1.35 * b),
    }
    count = int(np.count_nonzero(masks[color]))
    if count < minimum:
        raise AssertionError(f"{color} marker/forklift is not visible: {count} pixels")
    return count


def depth_metrics(path: Path, width: int, height: int, far_m: float) -> dict[str, Any]:
    depth = np.load(path)
    if depth.shape != (height, width):
        raise AssertionError(f"unexpected depth shape: {depth.shape}")
    finite = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
    if int(finite.sum()) < depth.size // 4:
        raise AssertionError("too few finite geometry depth pixels")
    return {
        "shape": list(depth.shape),
        "finite_geometry_pixels": int(finite.sum()),
        "min_m": float(depth[finite].min()),
        "max_m": float(depth[finite].max()),
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    backend = os.environ.get("MUJOCO_GL", "unset")
    if backend not in {"glfw", "cgl", "egl", "osmesa"}:
        raise AssertionError("render test requires MUJOCO_GL=glfw/cgl/egl/osmesa")
    env = build_environment(args.model, args.spec)
    width, height = (int(value) for value in env.spec["outputs"]["resolution"])

    before_info = env.capture_rgbd(args.screenshot_dir / "initial")
    before = load_rgb(env.package_root / before_info["rgb"]["path"], width, height)
    before_depth = depth_metrics(
        env.package_root / before_info["depth"]["path"],
        width,
        height,
        float(env.model.vis.map.zfar * env.model.stat.extent),
    )
    marker_pixels = {color: color_pixels(before, color) for color in ("yellow", "cyan", "green", "magenta", "orange")}
    before_position = np.asarray(env.observe()["forklift_position"], dtype=float)

    env.step({"id": "drive_to_pallet", "payload": {"speed_mps": 0.8}})
    env.step({"id": "raise_forks", "payload": {"lift_m": 0.18}})
    env.step({"id": "engage_pallet", "payload": {"confirm": True}})
    env.step({"id": "carry_to_drop_zone", "payload": {"speed_mps": 0.8}})
    env.step({"id": "lower_forks_release", "payload": {"release": True}})
    final_obs = env.step(
        {"id": "inspect_forklift_delivery", "payload": {"output_dir": str(args.screenshot_dir / "final")}}
    )
    if not env.is_success():
        raise AssertionError("full forklift delivery sequence did not succeed")
    final_info = final_obs["sensor"]
    final = load_rgb(env.package_root / final_info["rgb"]["path"], width, height)
    final_depth = depth_metrics(
        env.package_root / final_info["depth"]["path"],
        width,
        height,
        float(env.model.vis.map.zfar * env.model.stat.extent),
    )
    for color in ("yellow", "cyan", "green", "magenta", "orange"):
        color_pixels(final, color, 8)
    movement = float(np.linalg.norm(np.asarray(final_obs["forklift_position"], dtype=float) - before_position))
    if movement < 0.50:
        raise AssertionError(f"forklift did not visibly move enough: {movement:.3f} m")

    env.reset()
    reset_info = env.capture_rgbd(args.screenshot_dir / "reset")
    reset = load_rgb(env.package_root / reset_info["rgb"]["path"], width, height)
    color_pixels(reset, "orange", 8)
    if env.observe()["history"] or env.is_success():
        raise AssertionError("reset did not clear task state")

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "render_backend_requested": backend,
        "renderer_context": final_info["renderer_context"],
        "path_base": "package_root",
        "resolution": [width, height],
        "task_success": True,
        "interaction_sequence": final_obs["history"],
        "visible_interaction_markers": {"status": "PASS", "pixels": marker_pixels},
        "forklift_movement_m": movement,
        "screenshots": {
            "initial": {"path": before_info["rgb"]["path"]},
            "final": {"path": final_info["rgb"]["path"]},
            "reset": {"path": reset_info["rgb"]["path"]},
        },
        "depth": {"initial": before_depth, "final": final_depth},
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--screenshot-dir", type=Path, default=base / "output" / "screenshots")
    parser.add_argument("--result", type=Path, default=base / "output" / "render_results.json")
    args = parser.parse_args()
    args.model = args.model.resolve()
    args.spec = args.spec.resolve()
    args.screenshot_dir = args.screenshot_dir.resolve()
    args.result = args.result.resolve()
    try:
        result = run(args)
        exit_code = 0
    except Exception as exc:
        result = {
            "status": "FAIL",
            "mujoco_executed": True,
            "mujoco_version": getattr(mujoco, "__version__", "unknown"),
            "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"),
            "error_type": type(exc).__name__,
            "error": type(exc).__name__,
        }
        exit_code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
