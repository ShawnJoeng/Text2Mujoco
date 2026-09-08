#!/usr/bin/env python3
"""Offscreen RGB-D and visible interaction smoke test for MuJoCo."""

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


def check_rgb(
    path: Path, *, width: int, height: int
) -> tuple[np.ndarray, dict[str, Any]]:
    image = np.asarray(Image.open(path).convert("RGB"))
    if image.shape != (height, width, 3):
        raise AssertionError(f"unexpected screenshot shape: {image.shape}")
    standard_deviation = float(image.std())
    dynamic_range = int(image.max()) - int(image.min())
    if standard_deviation < 5.0 or dynamic_range < 50:
        raise AssertionError(
            f"screenshot is blank/nearly uniform: std={standard_deviation}, range={dynamic_range}"
        )
    return image, {
        "path": str(path.resolve()),
        "shape": list(image.shape),
        "mean": float(image.mean()),
        "std": standard_deviation,
        "dynamic_range": dynamic_range,
    }


def package_relative(path: str, package_root: Path) -> str:
    """Return a stable package-relative report path without host details."""
    resolved = Path(path).resolve()
    try:
        return str(resolved.relative_to(package_root.resolve()))
    except ValueError as exc:
        raise AssertionError("renderer returned a path outside the package") from exc


def resolve_package_path(path: str, package_root: Path) -> Path:
    value = Path(path)
    return value if value.is_absolute() else package_root / value


def portable_metrics(metrics: dict[str, Any], package_root: Path) -> dict[str, Any]:
    """Normalize a report metric path without exposing the local filesystem."""
    result = dict(metrics)
    if isinstance(result.get("path"), str):
        result["path"] = package_relative(result["path"], package_root)
    if isinstance(result.get("preview_path"), str):
        result["preview_path"] = package_relative(result["preview_path"], package_root)
    return result


def red_centroid(image: np.ndarray) -> tuple[np.ndarray, int]:
    red = image[..., 0].astype(float)
    green = image[..., 1].astype(float)
    blue = image[..., 2].astype(float)
    mask = (red > 80) & (red > 1.6 * green) & (red > 1.6 * blue)
    y, x = np.nonzero(mask)
    if len(x) < 50:
        raise AssertionError(f"red cube is not visibly detected: red_pixels={len(x)}")
    return np.asarray([float(x.mean()), float(y.mean())]), int(len(x))


def check_depth(
    path: Path, far_m: float, *, width: int, height: int
) -> tuple[np.ndarray, dict[str, Any]]:
    depth = np.load(path)
    if depth.shape != (height, width):
        raise AssertionError(f"unexpected depth shape: {depth.shape}")
    geometry_depth = (
        np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
    )
    if int(geometry_depth.sum()) < depth.size // 4:
        raise AssertionError(
            f"too few non-background depth pixels: {int(geometry_depth.sum())}"
        )
    return depth, {
        "path": str(path.resolve()),
        "shape": list(depth.shape),
        "finite_geometry_pixels": int(geometry_depth.sum()),
        "min_m": float(depth[geometry_depth].min()),
        "max_m": float(depth[geometry_depth].max()),
        "far_plane_m": far_m,
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    package_root = Path(__file__).resolve().parent
    backend = os.environ.get("MUJOCO_GL", "unset")
    if backend in {"unset", "disable"}:
        raise AssertionError("render_smoke.py requires an enabled MUJOCO_GL backend")
    environment = build_environment(args.model, args.spec)
    environment.run_physics(100)
    resolution = environment.spec["outputs"].get("resolution", [640, 480])
    width, height = int(resolution[0]), int(resolution[1])
    far_m = float(
        environment.model.vis.map.zfar * environment.model.stat.extent
    )

    initial = environment.capture_rgbd(args.screenshot_dir / "initial")
    initial_image, initial_metrics = check_rgb(
        resolve_package_path(initial["rgb"]["path"], package_root), width=width, height=height
    )
    initial_depth_frame, initial_depth = check_depth(
        resolve_package_path(initial["depth"]["path"], package_root), far_m, width=width, height=height
    )

    environment.reset()
    environment.step({"id": "press_start_button", "payload": {"press_depth_m": 0.018}})
    environment.step({"id": "grasp_red_cube", "payload": {"close": True}})
    environment.step(
        {
            "id": "place_cube_in_box",
            "payload": {"pose": [0.3, 0.18, 0.86], "release": True},
        }
    )
    final_observation = environment.step(
        {
            "id": "inspect_rgbd",
            "payload": {"output_dir": str(args.screenshot_dir / "final")},
        }
    )
    if not environment.is_success():
        raise AssertionError("full press -> grasp -> place -> inspect sequence did not succeed")
    final = final_observation["sensor"]
    final_image, final_metrics = check_rgb(
        resolve_package_path(final["rgb"]["path"], package_root), width=width, height=height
    )
    final_depth_frame, final_depth = check_depth(
        resolve_package_path(final["depth"]["path"], package_root), far_m, width=width, height=height
    )

    initial_center, initial_red_pixels = red_centroid(initial_image)
    final_center, final_red_pixels = red_centroid(final_image)
    movement = float(np.linalg.norm(final_center - initial_center))
    if movement <= 10.0:
        raise AssertionError(f"red cube did not visibly move enough: {movement:.3f}px")
    geometry = (initial_depth_frame < far_m * 0.999) | (
        final_depth_frame < far_m * 0.999
    )
    depth_changed = geometry & (
        np.abs(final_depth_frame - initial_depth_frame) > 1e-4
    )
    changed_depth_pixels = int(depth_changed.sum())
    if changed_depth_pixels < 100:
        raise AssertionError(
            f"too few depth pixels changed after interaction: {changed_depth_pixels}"
        )

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "path_base": "package_root",
        "render_backend_requested": backend,
        "renderer_context": final["renderer_context"],
        "renderer_verified": "PASS",
        "rgb_verified": "PASS",
        "depth_verified": "PASS",
        "interaction_sequence": final_observation["state"]["history"],
        "task_success": True,
        "red_cube": {
            "initial_centroid_px": initial_center.tolist(),
            "final_centroid_px": final_center.tolist(),
            "movement_px": movement,
            "initial_red_pixels": initial_red_pixels,
            "final_red_pixels": final_red_pixels,
        },
        "screenshots": {
            "initial": portable_metrics(initial_metrics, package_root),
            "final": portable_metrics(final_metrics, package_root),
        },
        "depth": {
            "initial": portable_metrics(initial_depth, package_root),
            "final": portable_metrics(final_depth, package_root),
            "changed_geometry_pixels": changed_depth_pixels,
        },
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument(
        "--screenshot-dir", type=Path, default=base / "output" / "screenshots"
    )
    parser.add_argument(
        "--result", type=Path, default=base / "output" / "render_results.json"
    )
    args = parser.parse_args()
    args.model = args.model.resolve()
    args.spec = args.spec.resolve()
    args.screenshot_dir = args.screenshot_dir.resolve()
    args.result = args.result.resolve()
    exit_code = 0
    try:
        result = run(args)
    except Exception as exc:
        exit_code = 1
        result = {
            "status": "FAIL",
            "mujoco_executed": True,
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "path_base": "package_root",
            "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"),
            "error_type": type(exc).__name__,
            "error": type(exc).__name__,
        }
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
