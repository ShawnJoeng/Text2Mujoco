#!/usr/bin/env python3
"""CGL top-camera validation for visible markers, navigation, and reset."""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import traceback
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
    if float(image.std()) < 8 or int(image.max()) - int(image.min()) < 80:
        raise AssertionError("RGB frame is blank or nearly uniform")
    return image


def color_centroid(image: np.ndarray, color: str, minimum_pixels: int = 30) -> tuple[np.ndarray, int]:
    r, g, b = (image[..., index].astype(float) for index in range(3))
    masks = {
        "orange": (r > 120) & (r > 1.5 * g) & (g > 25) & (g > 1.2 * b),
        "yellow": (r > 140) & (g > 105) & (r > 1.15 * b) & (g > 1.6 * b),
        "green": (g > 95) & (g > 1.45 * r) & (g > 1.45 * b),
        "magenta": (r > 100) & (b > 65) & (r > 1.35 * g) & (b > 1.2 * g),
    }
    y, x = np.nonzero(masks[color])
    if len(x) < minimum_pixels:
        raise AssertionError(f"{color} marker/object not visible: {len(x)} pixels")
    return np.array([float(x.mean()), float(y.mean())]), int(len(x))


def check_depth(path: Path, width: int, height: int, far_m: float) -> dict[str, Any]:
    depth = np.load(path)
    if depth.shape != (height, width):
        raise AssertionError(f"unexpected depth shape: {depth.shape}")
    geometry = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
    if int(geometry.sum()) < depth.size // 3:
        raise AssertionError("too few finite geometry depth pixels")
    return {
        "path": str(path.resolve()),
        "shape": list(depth.shape),
        "finite_geometry_pixels": int(geometry.sum()),
        "min_m": float(depth[geometry].min()),
        "max_m": float(depth[geometry].max()),
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    backend = os.environ.get("MUJOCO_GL", "unset")
    if backend not in {"glfw", "cgl"}:
        raise AssertionError("macOS render test requires MUJOCO_GL=glfw or cgl (native CGL context)")
    env = build_environment(args.model, args.spec)
    width, height = (int(value) for value in env.spec["outputs"]["resolution"])
    far_m = float(env.model.vis.map.zfar * env.model.stat.extent)

    before_info = env.capture_rgbd(args.screenshot_dir, "before")
    before = load_rgb(Path(before_info["rgb"]["path"]), width, height)
    before_depth = check_depth(Path(before_info["depth"]["path"]), width, height, far_m)
    before_robot, before_robot_pixels = color_centroid(before, "orange", 45)
    marker_pixels: dict[str, int] = {}
    marker_centroids: dict[str, list[float]] = {}
    for color in ("yellow", "green", "magenta"):
        centroid, count = color_centroid(before, color)
        marker_pixels[color] = count
        marker_centroids[color] = centroid.tolist()

    env.step({"id": "reach_checkpoint_a", "payload": {"speed_mps": 0.8}})
    env.step({"id": "reach_checkpoint_b", "payload": {"speed_mps": 0.8}})
    after_obs = env.step({"id": "inspect_top_camera", "payload": {"output_dir": str(args.screenshot_dir)}})
    if not env.is_success():
        raise AssertionError("A -> south bypass -> B -> camera sequence did not succeed")
    after_info = after_obs["sensor"]
    after = load_rgb(Path(after_info["rgb"]["path"]), width, height)
    after_depth = check_depth(Path(after_info["depth"]["path"]), width, height, far_m)
    after_robot, after_robot_pixels = color_centroid(after, "orange", 45)
    movement_px = float(np.linalg.norm(after_robot - before_robot))
    if movement_px < 120:
        raise AssertionError(f"robot did not visibly move enough: {movement_px:.2f} px")
    for color in ("yellow", "green", "magenta"):
        color_centroid(after, color, 20)

    env.reset()
    reset_info = env.capture_rgbd(args.screenshot_dir, "reset")
    reset = load_rgb(Path(reset_info["rgb"]["path"]), width, height)
    reset_robot, reset_robot_pixels = color_centroid(reset, "orange", 45)
    reset_error_px = float(np.linalg.norm(reset_robot - before_robot))
    if reset_error_px > 3.0:
        raise AssertionError(f"rendered reset did not restore robot: {reset_error_px:.2f} px")
    if env.is_success() or env.observe()["state"]["history"]:
        raise AssertionError("reset did not clear rendered task state")

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "command": " ".join(sys.argv),
        "render_backend_requested": backend,
        "renderer_context": "native macOS CGL via MuJoCo glfw backend" if backend == "glfw" else "native macOS CGL",
        "resolution": [width, height],
        "task_success": True,
        "interaction_sequence": after_obs["state"]["history"],
        "visible_interaction_markers": {"status": "PASS", "pixels": marker_pixels, "centroids_px": marker_centroids},
        "robot_pixel_movement": {
            "status": "PASS",
            "before_centroid_px": before_robot.tolist(),
            "after_centroid_px": after_robot.tolist(),
            "movement_px": movement_px,
            "before_pixels": before_robot_pixels,
            "after_pixels": after_robot_pixels,
        },
        "reset_render": {"status": "PASS", "centroid_error_px": reset_error_px, "reset_pixels": reset_robot_pixels},
        "screenshots": {"before": before_info["rgb"], "after": after_info["rgb"], "reset": reset_info["rgb"]},
        "depth": {"before": before_depth, "after": after_depth},
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
        result = run(args)
        exit_code = 0
    except Exception as exc:
        result = {
            "status": "FAIL",
            "mujoco_executed": True,
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "platform": platform.platform(),
            "command": " ".join(sys.argv),
            "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }
        exit_code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

