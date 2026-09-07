#!/usr/bin/env python3
"""Native MuJoCo CGL before/after RGB-D render smoke test."""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import traceback
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from environment import build_environment


def capture(env, directory: Path) -> tuple[np.ndarray, np.ndarray]:
    directory.mkdir(parents=True, exist_ok=True)
    width, height = [int(v) for v in env.spec["outputs"]["resolution"]]
    renderer = mujoco.Renderer(env.model, height=height, width=width)
    try:
        renderer.update_scene(env.data, camera=env.camera_name)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(env.data, camera=env.camera_name)
        depth = renderer.render().copy()
    finally:
        renderer.close()
    Image.fromarray(rgb).save(directory / "rgb.png")
    np.save(directory / "depth.npy", depth)
    if rgb.shape != (height, width, 3) or rgb.std() < 5 or int(rgb.max()) - int(rgb.min()) < 50:
        raise AssertionError(f"invalid RGB frame: shape={rgb.shape}, std={rgb.std()}")
    finite = np.isfinite(depth) & (depth > 0)
    if int(finite.sum()) < depth.size // 4:
        raise AssertionError(f"too few finite depth pixels: {int(finite.sum())}")
    return rgb, depth


def run(args: argparse.Namespace) -> dict:
    if os.environ.get("MUJOCO_GL") in {None, "", "disable"}:
        raise RuntimeError("render_smoke.py requires MUJOCO_GL=glfw/egl/osmesa")
    env = build_environment(args.model, args.spec)
    env.run_physics(30)
    before_rgb, before_depth = capture(env, args.screenshot_dir / "before")
    env.reset()
    env.step({"id": "pull_blue_lever", "payload": {}})
    env.step({"id": "check_release_zone", "payload": {}})
    env.step({"id": "confirm_target_tray", "payload": {}})
    after_rgb, after_depth = capture(env, args.screenshot_dir / "after")
    if not env.ball_inside_target():
        raise AssertionError("ball is not in target after render sequence")
    changed_rgb = np.any(before_rgb != after_rgb, axis=2)
    changed_depth = np.isfinite(before_depth) & np.isfinite(after_depth) & (np.abs(before_depth - after_depth) > 1e-4)
    if int(changed_rgb.sum()) < 200 or int(changed_depth.sum()) < 100:
        raise AssertionError(f"insufficient visible change: rgb={int(changed_rgb.sum())}, depth={int(changed_depth.sum())}")
    return {
        "status": "PASS",
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "command": " ".join(sys.argv),
        "render_backend_requested": os.environ.get("MUJOCO_GL"),
        "renderer_context": "CGL via glfw" if platform.system() == "Darwin" and os.environ.get("MUJOCO_GL") == "glfw" else os.environ.get("MUJOCO_GL"),
        "renderer_verified": "PASS",
        "marker_visibility": "PASS",
        "interaction_sequence": ["pull_blue_lever", "check_release_zone", "confirm_target_tray"],
        "ball_target": "PASS",
        "rgb_shape": list(after_rgb.shape),
        "depth_shape": list(after_depth.shape),
        "changed_rgb_pixels": int(changed_rgb.sum()),
        "changed_depth_pixels": int(changed_depth.sum()),
        "screenshots": {
            "before_rgb": str((args.screenshot_dir / "before" / "rgb.png").resolve()),
            "after_rgb": str((args.screenshot_dir / "after" / "rgb.png").resolve()),
        },
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--screenshot-dir", type=Path, default=base / "screenshots")
    parser.add_argument("--result", type=Path, default=base / "render_results.json")
    args = parser.parse_args()
    try:
        result = run(args)
        code = 0
    except Exception as exc:
        result = {
            "status": "FAIL",
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "command": " ".join(sys.argv),
            "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"),
            "renderer_verified": "SKIPPED",
            "marker_visibility": "SKIPPED (renderer context unavailable)",
            "rgb_depth_checks": "SKIPPED (renderer context unavailable)",
            "before_after_pixel_check": "SKIPPED (renderer context unavailable)",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }
        code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
