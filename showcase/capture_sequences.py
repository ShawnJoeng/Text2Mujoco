#!/usr/bin/env python3
"""Capture reproducible RGB-D frames for every showcase interaction sequence.

The keyframe path renders each state in a short-lived MuJoCo renderer, while the
dense path reuses one renderer per scene. Both paths are independent from
viewer timing and produce a paced GIF, a multi-page TIFF, and inspectable
machine-readable capture metadata.
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
from pathlib import Path
from typing import Any, Dict, List, Tuple

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parent


def require_repository_output_root(output_root: Path) -> Path:
    """Keep generated reports and sensor paths in the self-contained showcase tree."""
    try:
        resolved = Path(output_root).resolve()
    except (OSError, RuntimeError, ValueError) as exc:
        raise ValueError("output root is invalid") from exc
    if resolved != ROOT:
        raise ValueError("output root must be the repository showcase directory")
    return resolved


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
    "05-robot-arm-sorting": {
        "warmup_steps": 100,
        "camera": "sorting_camera",
        "actions": [
            ("approach_blue_part", {"speed_rad_s": 1.0}),
            ("grasp_blue_part", {"close": True}),
            ("transfer_to_blue_bin", {"speed_rad_s": 1.0}),
            ("release_blue_part", {"open": True}),
            ("inspect_sorting_result", {"output_dir": "runtime_inspect"}),
        ],
    },
    "06-forklift-pallet": {
        "warmup_steps": 100,
        "camera": "delivery_camera",
        "settle_after": {"lower_forks_release": 120},
        "actions": [
            ("drive_to_pallet", {"speed_mps": 0.8}),
            ("raise_forks", {"lift_m": 0.18}),
            ("engage_pallet", {"confirm": True}),
            ("carry_to_drop_zone", {"speed_mps": 0.8}),
            ("lower_forks_release", {"release": True}),
            ("inspect_forklift_delivery", {"output_dir": "runtime_inspect"}),
        ],
    },
    "07-robot-assembly": {
        "warmup_steps": 100,
        "camera": "assembly_camera",
        "settle_after": {"release_assembled_peg": 120},
        "actions": [
            ("move_arm_to_peg", {"speed_rad_s": 1.0}),
            ("grasp_peg_with_arm", {"close": True}),
            ("move_arm_to_socket", {"speed_rad_s": 1.0}),
            ("insert_peg_into_socket", {"depth_m": 0.08}),
            ("release_assembled_peg", {"open": True}),
            ("inspect_assembly", {"output_dir": "runtime_inspect"}),
        ],
    },
    "08-conveyor-arm": {
        "warmup_steps": 100,
        "camera": "handoff_camera",
        "settle_after": {"release_parcel_in_target_bin": 120},
        "actions": [
            ("start_conveyor_to_pickup", {"speed_mps": 0.6}),
            ("move_arm_to_parcel", {"speed_rad_s": 1.0}),
            ("grasp_parcel_with_arm", {"close": True}),
            ("move_arm_to_target_bin", {"speed_rad_s": 1.0}),
            ("release_parcel_in_target_bin", {"open": True}),
            ("inspect_handoff", {"output_dir": "runtime_inspect"}),
        ],
    },
}


def load_environment(
    scene_dir: Path,
    *,
    model_path: Path | None = None,
    spec_path: Path | None = None,
):
    module_path = scene_dir / "environment.py"
    module_name = "showcase_environment_" + scene_dir.name.replace("-", "_")
    module_spec = importlib.util.spec_from_file_location(module_name, module_path)
    if module_spec is None or module_spec.loader is None:
        raise RuntimeError("could not load " + str(module_path))
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    return module.build_environment(
        model_path or (scene_dir / "model.xml"),
        spec_path or (scene_dir / "scene_spec.json"),
    )


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
    label_h, gap = 34, 12
    # Use three columns for longer storyboards so five/six frames stay
    # compact; keep four-frame layouts at two columns for larger thumbnails.
    columns = 2 if len(frames) <= 4 else 3
    tile_w = (676 - (columns + 1) * gap) // columns
    tile_h = round(tile_w * 480 / 640)
    rows = (len(frames) + columns - 1) // columns
    sheet_width = columns * tile_w + (columns + 1) * gap
    sheet = Image.new("RGB", (sheet_width, rows * (tile_h + label_h) + (rows + 1) * gap), "#f4f6f8")
    draw = ImageDraw.Draw(sheet)
    for index, (frame, label) in enumerate(zip(frames, labels)):
        source = Image.open(frame).convert("RGB")
        resampling = getattr(Image, "Resampling", Image)
        thumb = ImageOps.contain(source, (tile_w, tile_h), method=resampling.LANCZOS)
        col, row = index % columns, index // columns
        row_start = row * columns
        row_count = min(columns, len(frames) - row_start)
        row_width = row_count * tile_w + (row_count - 1) * gap
        row_left = (sheet_width - row_width) // 2
        tile_left = row_left + col * (tile_w + gap)
        x = tile_left + (tile_w - thumb.width) // 2
        y = gap + row * (tile_h + label_h) + (tile_h - thumb.height) // 2
        sheet.paste(thumb, (x, y))
        label = label.replace("_", " ")
        label_size = 17
        label_font = font(label_size)
        while label_size > 11 and draw.textbbox((0, 0), label, font=label_font)[2] > tile_w - 4:
            label_size -= 1
            label_font = font(label_size)
        label_width = draw.textbbox((0, 0), label, font=label_font)[2]
        label_y = gap + row * (tile_h + label_h) + tile_h + 7
        draw.text((tile_left + (tile_w - label_width) // 2, label_y), label, fill="#16202a", font=label_font)
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


def write_gif(
    frames: List[Path],
    labels: List[str],
    output_path: Path,
    frame_duration_ms: int = 1600,
    final_duration_ms: int = 2600,
    max_size: tuple[int, int] | None = None,
) -> None:
    """Write a paced animation of the interaction keyframes."""
    if not frames:
        raise AssertionError("no frames to write")
    if len(frames) != len(labels):
        raise ValueError("GIF frames and labels must have the same length")
    if frame_duration_ms < 100 or final_duration_ms < 100:
        raise ValueError("GIF frame durations must be at least 100 ms")

    images = []
    for index, (frame, label) in enumerate(zip(frames, labels), start=1):
        source = Image.open(frame).convert("RGB")
        if max_size is not None:
            source.thumbnail(max_size, Image.Resampling.LANCZOS)
        label_height = 48
        canvas = Image.new("RGB", (source.width, source.height + label_height), "#f4f6f8")
        canvas.paste(source, (0, 0))
        draw = ImageDraw.Draw(canvas)
        clean_label = re.sub(r"^\d+\s+", "", label).replace("_", " ")
        caption = f"{index}/{len(frames)}  {clean_label}"
        caption_font = font(22)
        caption_width = draw.textbbox((0, 0), caption, font=caption_font)[2]
        draw.text(
            ((canvas.width - caption_width) // 2, source.height + 10),
            caption,
            fill="#16202a",
            font=caption_font,
        )
        images.append(canvas.convert("P", palette=Image.Palette.ADAPTIVE))
        source.close()

    durations = [frame_duration_ms] * len(images)
    durations[-1] = final_duration_ms
    output_path.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        output_path,
        save_all=True,
        append_images=images[1:],
        duration=durations,
        loop=0,
        optimize=False,
        disposal=2,
    )
    for image in images:
        image.close()


def verify_dense_archives(
    gif_path: Path,
    tiff_path: Path,
    expected_frame_count: int,
    frame_duration_ms: int,
    final_duration_ms: int,
    expected_size: Tuple[int, int],
) -> Dict[str, Any]:
    """Reopen dense archives and verify their frame counts and timing."""
    if expected_frame_count < 1:
        raise AssertionError("dense archive must contain at least one frame")
    expected_durations = [frame_duration_ms] * expected_frame_count
    expected_durations[-1] = final_duration_ms

    gif_durations: List[int] = []
    with Image.open(gif_path) as gif:
        gif_frame_count = int(getattr(gif, "n_frames", 1))
        for index in range(gif_frame_count):
            gif.seek(index)
            gif_durations.append(int(gif.info.get("duration", 0)))
            if float(np.asarray(gif.convert("RGB")).std()) < 5.0:
                raise AssertionError(f"dense GIF frame {index} is blank or nearly uniform")
    if gif_frame_count != expected_frame_count:
        raise AssertionError(f"dense GIF has {gif_frame_count} frames; expected {expected_frame_count}")
    if gif_durations != expected_durations:
        raise AssertionError(f"dense GIF timing mismatch: {gif_durations}")

    with Image.open(tiff_path) as tiff:
        tiff_page_count = int(getattr(tiff, "n_frames", 1))
        for index in range(tiff_page_count):
            tiff.seek(index)
            if tiff.size != expected_size:
                raise AssertionError(f"dense TIFF page {index} has unexpected size {tiff.size}")
            if float(np.asarray(tiff.convert("RGB")).std()) < 5.0:
                raise AssertionError(f"dense TIFF page {index} is blank or nearly uniform")
    if tiff_page_count != expected_frame_count:
        raise AssertionError(f"dense TIFF has {tiff_page_count} pages; expected {expected_frame_count}")

    return {
        "status": "PASS",
        "gif_frame_count": gif_frame_count,
        "tiff_page_count": tiff_page_count,
        "gif_frame_durations_ms": sorted(set(gif_durations)),
        "all_gif_frames_nonblank": True,
    }


class DenseSampler:
    """Capture real RGB states on simulation-time boundaries.

    MuJoCo scenes use different timesteps, so wall-clock sleeps are incorrect.
    The sampler wraps the module-level ``mujoco.mj_step`` used by the generated
    environments and records the first post-step state at or beyond each
    0.20-second simulation target. It never interpolates a physics frame.
    """

    def __init__(self, env: Any, scene_name: str, camera: str, output_dir: Path, interval_s: float = 0.2):
        self.env = env
        self.scene_name = scene_name
        self.camera = camera
        self.output_dir = Path(output_dir)
        self.interval_s = float(interval_s)
        if not np.isfinite(self.interval_s) or self.interval_s <= 0.0:
            raise ValueError("dense sampling interval must be a positive finite number")
        timestep = float(env.model.opt.timestep)
        if self.interval_s < timestep - 1e-12:
            raise ValueError(
                f"dense sampling interval ({self.interval_s:g}s) must be at least "
                f"the MuJoCo timestep ({timestep:g}s)"
            )
        self.origin_time_s = 0.0
        self.next_target_s = 0.0
        self.physics_steps = 0
        self.phase = "initial"
        self.frames: List[Path] = []
        self.labels: List[str] = []
        self.records: List[Dict[str, Any]] = []
        self.renderer: Any | None = None
        self._original_mj_step: Any | None = None
        self.paused = False

    def install(self) -> None:
        if self._original_mj_step is not None:
            raise RuntimeError("dense sampler is already installed")
        original = mujoco.mj_step
        self._original_mj_step = original
        sampler = self

        def wrapped(model: Any, data: Any, *args: Any, **kwargs: Any) -> None:
            if data is not sampler.env.data:
                original(model, data, *args, **kwargs)
                return

            # MuJoCo also accepts an optional nstep argument. Expand it so
            # each physical state is available to the simulation-time sampler
            # Preserve each physical step instead of duplicating the endpoint.
            nstep = kwargs.pop("nstep", None)
            remaining_args = args
            if nstep is None and len(args) == 1 and isinstance(args[0], int):
                nstep = args[0]
                remaining_args = ()
            if nstep is None:
                nstep = 1
            if not isinstance(nstep, int) or nstep < 1:
                raise ValueError("mj_step nstep must be a positive integer")
            for _ in range(nstep):
                original(model, data, *remaining_args, **kwargs)
                sampler.physics_steps += 1
                if not sampler.paused:
                    sampler._sample_due()

        mujoco.mj_step = wrapped

    def uninstall(self) -> None:
        if self._original_mj_step is not None:
            mujoco.mj_step = self._original_mj_step
            self._original_mj_step = None
        self.close_renderer()

    def close_renderer(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def _sample_due(self) -> None:
        elapsed = float(self.env.data.time) - self.origin_time_s
        # Drain every simulation-time boundary crossed by this physics call.
        # If the interval is smaller than the model timestep, multiple targets
        # can legitimately map to the same post-step state; the report keeps
        # both the target and actual timestamps so this is explicit.
        catch_up = 0
        while elapsed + 1e-9 >= self.next_target_s:
            target = self.next_target_s
            self.capture(target_time_s=target, event=False)
            self.next_target_s += self.interval_s
            catch_up += 1
            if catch_up > 10000:
                raise RuntimeError("dense sampler could not drain simulation-time boundaries")

    def _ensure_renderer(self) -> Any:
        if self.renderer is None:
            resolution = self.env.spec.get("outputs", {}).get("resolution", [640, 480])
            self.renderer = mujoco.Renderer(self.env.model, height=int(resolution[1]), width=int(resolution[0]))
        return self.renderer

    def capture(self, target_time_s: float | None, event: bool, label: str | None = None) -> None:
        renderer = self._ensure_renderer()
        renderer.update_scene(self.env.data, camera=self.camera)
        rgb = renderer.render().copy()
        if rgb.dtype != np.uint8 or rgb.std() < 5:
            raise AssertionError("dense RGB frame is blank or invalid")
        absolute_sim_time = float(self.env.data.time)
        sim_time = absolute_sim_time - self.origin_time_s
        rounded_absolute_time = round(absolute_sim_time, 6)
        rounded_sim_time = round(sim_time, 6)
        rounded_target_time = None if target_time_s is None else round(float(target_time_s), 6)
        index = len(self.frames)
        clean_label = label or self.phase
        safe_label = re.sub(r"[^a-zA-Z0-9]+", "_", clean_label).strip("_") or "state"
        frame_dir = self.output_dir / "frames"
        frame_dir.mkdir(parents=True, exist_ok=True)
        frame_path = frame_dir / f"frame_{index:04d}_{sim_time:08.3f}_{safe_label}.png"
        Image.fromarray(rgb).save(frame_path)
        self.frames.append(frame_path)
        self.labels.append(f"t={rounded_sim_time:.2f}s  {clean_label}")
        observation = compact_observation(self.scene_name, self.env.observe())
        self.records.append(
            {
                "index": index,
                "target_time_s": rounded_target_time,
                "actual_time_s": rounded_sim_time,
                "absolute_sim_time_s": rounded_absolute_time,
                "physics_step": self.physics_steps,
                "phase": clean_label,
                "event_frame": bool(event),
                "rgb": str(frame_path),
                "rgb_shape": list(rgb.shape),
                "observation": observation,
            }
        )

    def start(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.origin_time_s = float(self.env.data.time)
        self.capture(target_time_s=0.0, event=False, label="initial")
        self.next_target_s = self.interval_s

    def set_phase(self, phase: str) -> None:
        self.phase = phase

    def pause(self) -> None:
        self.paused = True
        self.close_renderer()

    def resume(self) -> None:
        self.paused = False

    def finish(self) -> None:
        self.close_renderer()

    def write_outputs(self, archive_scene_dir: Path) -> Dict[str, Any]:
        gif_path = self.output_dir.parent / "dense_sequence.gif"
        tiff_path = self.output_dir.parent / "dense_sequence.tif"
        frame_duration_ms = 200
        final_duration_ms = 800
        write_gif(
            self.frames,
            self.labels,
            gif_path,
            frame_duration_ms=frame_duration_ms,
            final_duration_ms=final_duration_ms,
            max_size=(640, 480),
        )
        write_tiff(self.frames, tiff_path)
        with Image.open(self.frames[0]) as first_frame:
            expected_size = first_frame.size
        archive_verification = verify_dense_archives(
            gif_path,
            tiff_path,
            len(self.frames),
            frame_duration_ms,
            final_duration_ms,
            expected_size,
        )
        # The dense source PNGs are temporary. ``archive_frame``
        # is the stable one-based page number shared by the GIF and TIFF.
        regular_frame_count = sum(not bool(record["event_frame"]) for record in self.records)
        event_frame_count = len(self.records) - regular_frame_count
        report = {
            "status": "PASS",
            "scene": self.scene_name,
            "path_base": "package_root",
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"),
            "renderer_context": "native macOS CGL via glfw" if platform.system() == "Darwin" and os.environ.get("MUJOCO_GL") == "glfw" else os.environ.get("MUJOCO_GL", "unset"),
            "sequence_kind": "dense simulation frames",
            "simulation_interval_s": self.interval_s,
            "sampling_clock": "MuJoCo data.time elapsed since reset",
            "sampling_policy": "first post-step state at or beyond each target; action-boundary event frames are extra",
            "gif_frame_duration_ms": frame_duration_ms,
            "gif_final_frame_duration_ms": final_duration_ms,
            "depth_capture": "keyframes_only; dense pass stores RGB frames",
            "frame_count": len(self.frames),
            "regular_frame_count": regular_frame_count,
            "event_frame_count": event_frame_count,
            "simulation_time_span_s": self.records[-1]["actual_time_s"] if self.records else 0.0,
            "gif": str(gif_path.relative_to(archive_scene_dir)),
            "tiff": str(tiff_path.relative_to(archive_scene_dir)),
            "frame_storage": "GIF and TIFF only; dense source PNGs are removed after encoding",
            "frames": [
                {
                    # ``index`` remains zero-based for programmatic consumers;
                    # ``archive_frame`` is the stable one-based frame/page
                    # number shared by the GIF and TIFF archives.
                    "archive_frame": int(record["index"]) + 1,
                    **{
                        key: value
                        for key, value in record.items()
                        if key != "rgb"
                    },
                }
                for record in self.records
            ],
            "archive_verification": archive_verification,
            "task_success": bool(self.env.is_success()),
        }
        report_path = archive_scene_dir / "output" / "dense_sequence_results.json"
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        clear_generated_dense_capture(self.output_dir)
        try:
            self.output_dir.rmdir()
        except OSError:
            # Keep unrelated user files beside the generated capture.
            pass
        return report


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
    if scene_name == "05-robot-arm-sorting":
        return {
            "history": observation["history"],
            "arm_state": observation["state"]["arm_state"],
            "gripper_state": observation["state"]["gripper_state"],
            "blue_part_state": observation["state"]["blue_part_state"],
            "arm_tcp_position": observation["arm_tcp_position"],
            "blue_part_position": observation["blue_part_position"],
            "blue_part_inside_bin": observation["blue_part_inside_bin"],
        }
    if scene_name == "06-forklift-pallet":
        return {
            "history": observation["history"],
            "forklift_state": observation["state"]["forklift_state"],
            "pallet_state": observation["state"]["pallet_state"],
            "forklift_position": observation["forklift_position"],
            "fork_lift_qpos_m": observation["fork_lift_qpos_m"],
            "pallet_position": observation["pallet_position"],
            "pallet_inside_delivery": observation["pallet_inside_delivery"],
            "rack_collision_count": observation["rack_collision_count"],
        }
    if scene_name == "07-robot-assembly":
        return {
            "history": observation["history"],
            "arm_state": observation["state"]["arm_state"],
            "gripper_state": observation["state"]["gripper_state"],
            "tool_state": observation["state"]["tool_state"],
            "peg_state": observation["state"]["peg_state"],
            "tool_position": observation["tool_position"],
            "peg_position": observation["peg_position"],
            "peg_socket_error_m": observation["peg_socket_error_m"],
        }
    if scene_name == "08-conveyor-arm":
        return {
            "history": observation["history"],
            "conveyor_state": observation["state"]["conveyor_state"],
            "arm_state": observation["state"]["arm_state"],
            "gripper_state": observation["state"]["gripper_state"],
            "parcel_state": observation["state"]["parcel_state"],
            "tool_position": observation["tool_position"],
            "parcel_position": observation["parcel_position"],
            "parcel_inside_target_bin": observation["parcel_inside_target_bin"],
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
            # Never follow a user-created symlink during cleanup.
            if child.is_symlink():
                child.unlink()
            elif child.is_dir():
                entries = {entry.name for entry in child.iterdir()}
                if entries and entries <= {"rgb.png", "depth.npy", "depth_preview.png"}:
                    shutil.rmtree(child)
            else:
                if child.suffix in {".png", ".npy"}:
                    child.unlink()


def clear_generated_dense_capture(dense_dir: Path) -> None:
    """Remove only collector-owned dense capture subdirectories."""
    if not dense_dir.exists():
        return
    if dense_dir.is_symlink() or not dense_dir.is_dir():
        raise RuntimeError("dense capture path must be a real directory: " + str(dense_dir))
    generated_frame = re.compile(r"^frame_\d+_[0-9.]+_.+\.png$")
    for name in ("frames", "sensor"):
        child = dense_dir / name
        if not child.exists() and not child.is_symlink():
            continue
        if child.is_symlink():
            child.unlink()
        elif child.is_dir():
            for entry in child.iterdir():
                if name == "frames" and generated_frame.fullmatch(entry.name):
                    if entry.is_file():
                        entry.unlink()
                elif name == "sensor" and entry.name in {
                    "rgb.png",
                    "depth.npy",
                    "depth_preview.png",
                    "before.png",
                    "after.png",
                    "before_depth.npy",
                    "after_depth.npy",
                }:
                    if entry.is_file():
                        entry.unlink()
            try:
                child.rmdir()
            except OSError:
                pass
        else:
            child.unlink()


def run_dense_scene(
    scene_name: str,
    scene_cfg: Dict[str, Any],
    output_root: Path,
    interval_s: float = 0.2,
) -> Dict[str, Any]:
    """Capture RGB at 0.20-second simulation-time intervals.

    The dense pass is separate from the keyframe pass: it keeps
    the full-resolution TIFF/contact sheet compact while giving the README a
    readable, timed animation of the actual intermediate simulator states.
    """
    output_root = require_repository_output_root(output_root)
    scene_dir = ROOT / scene_name
    screenshot_dir = output_root / scene_name / "output" / "screenshots"
    dense_dir = screenshot_dir / "dense"
    clear_generated_dense_capture(dense_dir)
    dense_gif = screenshot_dir / "dense_sequence.gif"
    dense_tiff = screenshot_dir / "dense_sequence.tif"
    dense_report_path = output_root / scene_name / "output" / "dense_sequence_results.json"
    for path in (dense_gif, dense_tiff, dense_report_path):
        if path.exists():
            path.unlink()

    env = load_environment(scene_dir)
    sampler = DenseSampler(env, scene_name, scene_cfg["camera"], dense_dir, interval_s=interval_s)
    sampler.install()
    try:
        env.reset()
        sampler.start()
        sampler.set_phase("warmup")
        if scene_cfg["warmup_steps"]:
            env.run_physics(int(scene_cfg["warmup_steps"]))

        for action_id, payload in scene_cfg["actions"]:
            action_payload = dict(payload)
            if "output_dir" in action_payload:
                action_payload["output_dir"] = str(dense_dir / "sensor")
            sampler.set_phase(action_id)
            if action_id.startswith("inspect_") or action_id == "inspect_rgbd":
                sampler.pause()
                env.step({"id": action_id, "payload": action_payload})
                sampler.resume()
                sampler.capture(target_time_s=None, event=True, label=action_id)
            else:
                env.step({"id": action_id, "payload": action_payload})
                sampler.capture(target_time_s=None, event=True, label=action_id)

            settle_steps = int(scene_cfg.get("settle_after", {}).get(action_id, 0))
            if settle_steps:
                sampler.set_phase("settled_after_" + action_id)
                env.run_physics(settle_steps)

        if not env.is_success():
            raise AssertionError(scene_name + " dense interaction sequence did not succeed")
        sampler.finish()
        report = sampler.write_outputs(output_root / scene_name)
    finally:
        sampler.uninstall()
    return report


def run_scene(
    scene_name: str,
    scene_cfg: Dict[str, Any],
    output_root: Path,
    *,
    model_path: Path | None = None,
    spec_path: Path | None = None,
) -> Dict[str, Any]:
    output_root = require_repository_output_root(output_root)
    scene_dir = ROOT / scene_name
    scene_output_dir = output_root / scene_name / "output"
    screenshot_dir = scene_output_dir / "screenshots"
    sequence_dir = screenshot_dir / "sequence"
    sequence_dir.mkdir(parents=True, exist_ok=True)
    clear_generated_sequence_frames(sequence_dir)
    env = load_environment(scene_dir, model_path=model_path, spec_path=spec_path)

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
    gif_path = screenshot_dir / "sequence.gif"
    write_contact_sheet(frame_paths, labels, contact_sheet)
    write_tiff(frame_paths, tiff_path)
    write_gif(frame_paths, labels, gif_path)
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
        "path_base": "package_root",
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "render_backend_requested": os.environ.get("MUJOCO_GL", "unset"),
        "renderer_context": "native macOS CGL via glfw" if platform.system() == "Darwin" and os.environ.get("MUJOCO_GL") == "glfw" else os.environ.get("MUJOCO_GL", "unset"),
        "interaction_sequence": [item[0] for item in scene_cfg["actions"]],
        "frame_count": len(frame_paths),
        "frames": frames,
        "contact_sheet": str(contact_sheet.relative_to(scene_dir)),
        "tiff": str(tiff_path.relative_to(scene_dir)),
        "gif": str(gif_path.relative_to(scene_dir)),
        "sequence_kind": "discrete interaction keyframes",
        "gif_frame_duration_ms": 1600,
        "gif_final_frame_duration_ms": 2600,
        "tiff_playback_timing": "unspecified; TIFF is a keyframe archive, not a timed animation",
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
    parser.add_argument(
        "--dense",
        action="store_true",
        help="capture real RGB states every 0.20 seconds of MuJoCo simulation time",
    )
    parser.add_argument(
        "--dense-interval",
        type=float,
        default=0.2,
        help="simulation-time interval in seconds for --dense (default: 0.2)",
    )
    args = parser.parse_args()
    args.output_root = require_repository_output_root(args.output_root)
    backend = os.environ.get("MUJOCO_GL", "")
    if backend in {"", "disable"}:
        raise SystemExit("capture_sequences.py requires MUJOCO_GL=glfw/egl/osmesa")
    selected = sorted(SCENES) if args.scene == "all" else [args.scene]
    reports: List[Dict[str, Any]] = []
    try:
        for scene_name in selected:
            if args.dense:
                reports.append(
                    run_dense_scene(
                        scene_name,
                        SCENES[scene_name],
                        args.output_root,
                        interval_s=args.dense_interval,
                    )
                )
            else:
                reports.append(run_scene(scene_name, SCENES[scene_name], args.output_root))
    except Exception as exc:
        report = {
            "status": "FAIL",
            "scene": selected[len(reports)],
            "mujoco_version": getattr(mujoco, "__version__", "unknown"),
            "render_backend_requested": backend,
            "error_type": type(exc).__name__,
            "error": type(exc).__name__,
        }
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 1
    print(json.dumps(reports, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
