#!/usr/bin/env python3
"""Capture reproducible RGB-D frames for every showcase interaction sequence.

The script deliberately renders each state in a fresh, short-lived MuJoCo
renderer. This keeps the capture independent from viewer timing and produces
both a multi-page TIFF (full-size frames) and a small PNG contact sheet for
README pages.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import re
import shutil
import sys
import traceback
from pathlib import Path
from typing import Any, Dict, List, Tuple

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parent


SCENES: Dict[str, Dict[str, Any]] = {
    "01-button-cube-box": {
        "warmup_steps": 100,
        "camera": "overhead_rgbd",
        "actions": [
            ("press_start_button", {"press_depth_m": 0.018}),
            ("grasp_red_cube", {"close": True}),
            ("place_cube_in_box", {"pose": [0.3, 0.18, 0.86], "release": True}),
            ("inspect_rgbd", {"output_dir": "runtime_inspect"}),
        ],
    },
    "02-smart-drawer": {
        "warmup_steps": 100,
        "camera": "fixed_inspection_camera",
        "settle_after": {"pull_drawer_22cm": 120},
        "actions": [
            ("press_unlock_button", {"press_depth_m": 0.012}),
            ("pull_drawer_22cm", {"distance_m": 0.22}),
            ("inspect_open_drawer", {"output_dir": "runtime_inspect"}),
        ],
    },
    "03-warehouse-navigation": {
        "warmup_steps": 0,
        "camera": "top_camera",
        "actions": [
            ("reach_checkpoint_a", {"speed_mps": 0.8}),
            ("reach_checkpoint_b", {"speed_mps": 0.8}),
            ("inspect_top_camera", {"output_dir": "runtime_inspect"}),
        ],
    },
    "04-lever-ball-ramp": {
        "warmup_steps": 30,
        "camera": "inspection_camera",
        "settle_after": {"confirm_target_tray": 120},
        "actions": [
            ("pull_blue_lever", {}),
            ("check_release_zone", {}),
            ("confirm_target_tray", {}),
            ("inspect_with_camera", {"output_dir": "runtime_inspect"}),
        ],
    },
}


def load_environment(scene_dir: Path):
    module_path = scene_dir / "environment.py"
    module_name = "showcase_environment_" + scene_dir.name.replace("-", "_")
    module_spec = importlib.util.spec_from_file_location(module_name, module_path)
    if module_spec is None or module_spec.loader is None:
        raise RuntimeError("could not load " + str(module_path))
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    return module.build_environment(scene_dir / "model.xml", scene_dir / "scene_spec.json")


def capture_frame(env: Any, camera: str, frame_dir: Path) -> Tuple[np.ndarray, np.ndarray]:
    frame_dir.mkdir(parents=True, exist_ok=True)
    resolution = env.spec.get("outputs", {}).get("resolution", [640, 480])
    width, height = int(resolution[0]), int(resolution[1])
    renderer = mujoco.Renderer(env.model, height=height, width=width)
    try:
        renderer.update_scene(env.data, camera=camera)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(env.data, camera=camera)
        depth = renderer.render().copy()
    finally:
        renderer.close()

    if rgb.shape != (height, width, 3) or rgb.dtype != np.uint8:
        raise AssertionError("invalid RGB frame: " + repr(rgb.shape))
    if float(rgb.std()) < 5.0 or int(rgb.max()) - int(rgb.min()) < 50:
        raise AssertionError("RGB frame is blank or nearly uniform")
    far_m = float(env.model.vis.map.zfar * env.model.stat.extent)
    finite = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
    if int(finite.sum()) < depth.size // 4:
        raise AssertionError("too few finite geometry depth pixels: " + str(int(finite.sum())))

    Image.fromarray(rgb).save(frame_dir / "rgb.png")
    np.save(frame_dir / "depth.npy", depth)
    return rgb, depth


def font(size: int) -> ImageFont.ImageFont:
    candidates = [
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for candidate in candidates:
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default()


def write_contact_sheet(frames: List[Path], labels: List[str], output_path: Path) -> None:
    tile_w, tile_h = 320, 240
    label_h, gap = 34, 12
    columns = 2
    rows = (len(frames) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * tile_w + (columns + 1) * gap, rows * (tile_h + label_h) + (rows + 1) * gap), "#f4f6f8")
    draw = ImageDraw.Draw(sheet)
    label_font = font(17)
    for index, (frame, label) in enumerate(zip(frames, labels)):
        source = Image.open(frame).convert("RGB")
        resampling = getattr(Image, "Resampling", Image)
        thumb = ImageOps.contain(source, (tile_w, tile_h), method=resampling.LANCZOS)
        col, row = index % columns, index // columns
        x = gap + col * tile_w + (tile_w - thumb.width) // 2
        y = gap + row * (tile_h + label_h) + (tile_h - thumb.height) // 2
        sheet.paste(thumb, (x, y))
        label_y = gap + row * (tile_h + label_h) + tile_h + 7
        draw.text((gap + col * tile_w, label_y), label, fill="#16202a", font=label_font)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output_path, "PNG", optimize=True)


def write_tiff(frames: List[Path], output_path: Path) -> None:
    images = [Image.open(frame).convert("RGB") for frame in frames]
    if not images:
        raise AssertionError("no frames to write")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(output_path, save_all=True, append_images=images[1:], compression="tiff_lzw")
    for image in images:
        image.close()


def compact_observation(scene_name: str, observation: Dict[str, Any]) -> Dict[str, Any]:
    if scene_name == "01-button-cube-box":
        return {
            "history": observation["state"]["history"],
            "button_state": observation["state"]["button_state"],
            "cube_state": observation["state"]["cube_state"],
            "button_joint_qpos": observation["button_joint_qpos"],
            "cube_position": observation["cube_position"],
            "cube_speed_mps": observation["cube_linear_speed_mps"],
            "cube_contacts_box_bottom": observation["cube_contacts_box_bottom"],
        }
    if scene_name == "02-smart-drawer":
        return {
            "history": observation["state"]["history"],
            "lock_state": observation["state"]["lock_state"],
            "drawer_state": observation["state"]["drawer_state"],
            "inspection_state": observation["state"]["inspection_state"],
            "drawer_joint_qpos": observation["drawer_joint_qpos"],
            "drawer_body_position": observation["drawer_body_position"],
        }
    if scene_name == "03-warehouse-navigation":
        return {
            "history": observation["state"]["history"],
            "robot_state": observation["state"]["robot_state"],
            "robot_position": observation["robot_position"],
            "distance_to_a_m": observation["distance_to_a_m"],
            "distance_to_b_m": observation["distance_to_b_m"],
            "shelf_collision_count": observation["state"]["shelf_collision_count"],
            "camera_inspection_complete": observation["state"]["camera_inspection_complete"],
        }
    if scene_name == "04-lever-ball-ramp":
        return {
            "history": observation["history"],
            "lever_state": observation["state"]["lever_state"],
            "gate_state": observation["state"]["gate_state"],
            "target_state": observation["state"]["target_state"],
            "inspection_state": observation["state"]["inspection_state"],
            "lever_angle_rad": observation["lever_angle_rad"],
            "gate_lift_m": observation["gate_lift_m"],
            "ball_position": observation["ball_position"],
            "ball_speed_mps": observation["ball_speed_mps"],
            "ball_inside_target": observation["ball_inside_target"],
        }
    raise KeyError(scene_name)


def clear_generated_sequence_frames(sequence_dir: Path) -> None:
    """Remove only files previously produced by this collector.

    Keeping cleanup narrow makes reruns deterministic without deleting a
    user's unrelated notes or assets placed beside the generated frames.
    """
    if not sequence_dir.exists():
        return
    generated = re.compile(r"^(?:\d{2}_|frame_\d{2}_)")
    for child in sequence_dir.iterdir():
        if child.name == "runtime_inspect" or child.name == "sequence_results.json" or generated.match(child.name):
            if child.is_dir():
                shutil.rmtree(child)
            else:
                child.unlink()


def run_scene(scene_name: str, scene_cfg: Dict[str, Any], output_root: Path) -> Dict[str, Any]:
    scene_dir = ROOT / scene_name
    scene_output_dir = output_root / scene_name / "output"
    screenshot_dir = scene_output_dir / "screenshots"
    sequence_dir = screenshot_dir / "sequence"
    sequence_dir.mkdir(parents=True, exist_ok=True)
    clear_generated_sequence_frames(sequence_dir)
    env = load_environment(scene_dir)

    # Establish the same settled initial state used by the scene's render test.
    env.reset()
    if scene_cfg["warmup_steps"]:
        env.run_physics(int(scene_cfg["warmup_steps"]))
    frame_paths: List[Path] = []
    labels: List[str] = []
    frames: List[Dict[str, Any]] = []
    depth_frames: List[np.ndarray] = []

    def save_state(index: int, label: str) -> None:
        frame_dir = sequence_dir / ("%02d_" % index + label)
        rgb, depth = capture_frame(env, scene_cfg["camera"], frame_dir)
        frame_paths.append(frame_dir / "rgb.png")
        labels.append("%02d  %s" % (index, label.replace("_", " ")))
        far_m = float(env.model.vis.map.zfar * env.model.stat.extent)
        finite_depth = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
        depth_frames.append(depth)
        frames.append(
            {
                "index": index,
                "label": label,
                "rgb": str((frame_dir / "rgb.png").relative_to(scene_dir)),
                "depth": str((frame_dir / "depth.npy").relative_to(scene_dir)),
                "rgb_shape": list(rgb.shape),
                "rgb_std": float(rgb.std()),
                "rgb_dynamic_range": int(rgb.max()) - int(rgb.min()),
                "depth_shape": list(depth.shape),
                "finite_depth_pixels": int(finite_depth.sum()),
                "finite_depth_min_m": float(depth[finite_depth].min()),
                "finite_depth_max_m": float(depth[finite_depth].max()),
                "observation": compact_observation(scene_name, env.observe()),
            }
        )

    save_state(0, "initial")
    # Reset after the initial warmup so the action sequence starts deterministically.
    env.reset()
    if scene_cfg["warmup_steps"]:
        env.run_physics(int(scene_cfg["warmup_steps"]))
    frame_index = 1
    for action_id, payload in scene_cfg["actions"]:
        action_payload = dict(payload)
        if "output_dir" in action_payload:
            action_payload["output_dir"] = str(sequence_dir / action_payload["output_dir"])
        env.step({"id": action_id, "payload": action_payload})
        save_state(frame_index, action_id)
        frame_index += 1
        settle_steps = int(scene_cfg.get("settle_after", {}).get(action_id, 0))
        if settle_steps:
            env.run_physics(settle_steps)
            save_state(frame_index, "settled_after_" + action_id)
            frame_index += 1

    if not env.is_success():
        raise AssertionError(scene_name + " interaction sequence did not succeed")
    # Keep the two README-facing files at a stable path; raw frames remain
    # under sequence/ so they can be inspected or regenerated independently.
    contact_sheet = screenshot_dir / "sequence.png"
    tiff_path = screenshot_dir / "sequence.tif"
    write_contact_sheet(frame_paths, labels, contact_sheet)
    write_tiff(frame_paths, tiff_path)
    shutil.copy2(frame_paths[0], screenshot_dir / "before.png")
    shutil.copy2(frame_paths[-1], screenshot_dir / "after.png")
    np.save(screenshot_dir / "before_depth.npy", depth_frames[0])
    np.save(screenshot_dir / "after_depth.npy", depth_frames[-1])
    runtime_capture = sequence_dir / "runtime_inspect"
    if runtime_capture.exists():
        shutil.rmtree(runtime_capture)
    result = {
        "status": "PASS",
        "scene": scene_name,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"),
        "renderer_context": "native macOS CGL via glfw" if platform.system() == "Darwin" and os.environ.get("MUJOCO_GL") == "glfw" else os.environ.get("MUJOCO_GL", "unset"),
        "interaction_sequence": [item[0] for item in scene_cfg["actions"]],
        "frame_count": len(frame_paths),
        "frames": frames,
        "contact_sheet": str(contact_sheet.relative_to(scene_dir)),
        "tiff": str(tiff_path.relative_to(scene_dir)),
        "before": str((screenshot_dir / "before.png").relative_to(scene_dir)),
        "after": str((screenshot_dir / "after.png").relative_to(scene_dir)),
        "before_depth": str((screenshot_dir / "before_depth.npy").relative_to(scene_dir)),
        "after_depth": str((screenshot_dir / "after_depth.npy").relative_to(scene_dir)),
        "task_success": True,
    }
    (scene_output_dir / "sequence_results.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=["all"] + sorted(SCENES), default="all")
    parser.add_argument("--output-root", type=Path, default=ROOT)
    args = parser.parse_args()
    backend = os.environ.get("MUJOCO_GL", "")
    if backend in {"", "disable"}:
        raise SystemExit("capture_sequences.py requires MUJOCO_GL=glfw/egl/osmesa")
    selected = sorted(SCENES) if args.scene == "all" else [args.scene]
    reports: List[Dict[str, Any]] = []
    try:
        for scene_name in selected:
            reports.append(run_scene(scene_name, SCENES[scene_name], args.output_root))
    except Exception as exc:
        report = {
            "status": "FAIL",
            "scene": selected[len(reports)],
            "mujoco_version": getattr(mujoco, "__version__", "unknown"),
            "render_backend_requested": backend,
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 1
    print(json.dumps(reports, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
