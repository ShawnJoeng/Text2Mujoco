#!/usr/bin/env python3
"""RGB-D rendering and visible interaction smoke test for showcase 05."""

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


def resolve_package(path: str | Path, package_root: Path) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = package_root / candidate
    resolved = candidate.resolve()
    resolved.relative_to(package_root.resolve())
    return resolved


def load_rgb(path: Path, width: int, height: int) -> np.ndarray:
    image = np.asarray(Image.open(path).convert("RGB"))
    if image.shape != (height, width, 3):
        raise AssertionError(f"unexpected RGB frame shape: {image.shape}")
    if float(image.std()) < 5.0 or int(image.max()) - int(image.min()) < 50:
        raise AssertionError("RGB frame is blank or nearly uniform")
    return image


def load_depth(path: Path, width: int, height: int, far_m: float) -> dict[str, Any]:
    depth = np.load(path)
    if depth.shape != (height, width):
        raise AssertionError(f"unexpected depth frame shape: {depth.shape}")
    valid = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
    if int(valid.sum()) < depth.size // 4:
        raise AssertionError("too few finite geometry depth pixels")
    return {"shape": list(depth.shape), "finite_geometry_pixels": int(valid.sum()), "min_m": float(depth[valid].min()), "max_m": float(depth[valid].max())}


def color_pixels(image: np.ndarray) -> dict[str, int]:
    r, g, b = (image[..., index].astype(float) for index in range(3))
    masks = {
        "blue": (b > 110) & (b > 1.35 * r) & (b > 1.2 * g),
        "red": (r > 120) & (r > 1.55 * g) & (r > 1.55 * b),
        "yellow": (r > 150) & (g > 120) & (b < 100),
        "cyan": (g > 130) & (b > 120) & (r < 100),
        "magenta": (r > 130) & (b > 100) & (g < 110),
        "green": (g > 120) & (g > 1.35 * r) & (g > 1.2 * b),
    }
    return {name: int(mask.sum()) for name, mask in masks.items()}


def run(args: argparse.Namespace) -> dict[str, Any]:
    backend = os.environ.get("MUJOCO_GL", "unset")
    if backend not in {"glfw", "cgl", "egl", "osmesa"}:
        raise AssertionError("render test requires MUJOCO_GL=glfw/cgl/egl/osmesa")
    package_root = Path(__file__).resolve().parent
    env = build_environment(args.model, args.spec)
    width, height = (int(value) for value in env.spec["outputs"]["resolution"])
    far_m = float(env.model.vis.map.zfar * env.model.stat.extent)
    before_info = env.capture_rgbd(args.screenshot_dir / "before")
    before_rgb_path = resolve_package(before_info["rgb"]["path"], package_root)
    before_depth_path = resolve_package(before_info["depth"]["path"], package_root)
    before_rgb = load_rgb(before_rgb_path, width, height)
    before_depth = load_depth(before_depth_path, width, height, far_m)
    before_colors = color_pixels(before_rgb)
    if before_colors["blue"] < 100 or before_colors["red"] < 100:
        raise AssertionError("colored robot/object geometry is not visible")
    if any(before_colors[name] < 20 for name in ("yellow", "cyan", "magenta", "green")):
        raise AssertionError("one or more interaction marker colors are not visible")
    initial_object = np.asarray(env.observe()["blue_part_position"], dtype=float)

    env.step({"id": "approach_blue_part", "payload": {"speed_rad_s": 1.0}})
    env.step({"id": "grasp_blue_part", "payload": {"close": True}})
    env.step({"id": "transfer_to_blue_bin", "payload": {"speed_rad_s": 1.0}})
    env.step({"id": "release_blue_part", "payload": {"open": True}})
    final_observation = env.step({"id": "inspect_sorting_result", "payload": {"output_dir": str(args.screenshot_dir / "after")}})
    if not env.is_success():
        raise AssertionError("complete arm sorting sequence did not succeed")
    final_sensor = final_observation["sensor"]
    final_rgb = load_rgb(resolve_package(final_sensor["rgb"]["path"], package_root), width, height)
    final_depth = load_depth(resolve_package(final_sensor["depth"]["path"], package_root), width, height, far_m)
    final_colors = color_pixels(final_rgb)
    final_object = np.asarray(final_observation["blue_part_position"], dtype=float)
    object_motion = float(np.linalg.norm(final_object - initial_object))
    if object_motion < 0.20:
        raise AssertionError(f"blue part did not move enough: {object_motion:.3f} m")
    rgb_delta = float(np.mean(np.abs(final_rgb.astype(float) - before_rgb.astype(float))))
    if rgb_delta < 2.0:
        raise AssertionError("before/after render did not change enough")
    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "path_base": "package_root",
        "render_backend_requested": backend,
        "renderer_context": final_sensor.get("renderer_context", "unknown"),
        "rgb_verified": "PASS",
        "depth_verified": "PASS",
        "visible_markers": before_colors,
        "interaction_sequence": final_observation["history"],
        "task_success": True,
        "blue_part_motion_m": object_motion,
        "rgb_mean_absolute_delta": rgb_delta,
        "depth": {"before": before_depth, "after": final_depth},
        "screenshots": {"before": before_info["rgb"], "after": final_sensor["rgb"]},
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
        code = 0
    except Exception as exc:
        result = {"status": "FAIL", "mujoco_executed": True, "mujoco_version": mujoco.__version__, "path_base": "package_root", "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"), "error_type": type(exc).__name__, "error": type(exc).__name__}
        code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
