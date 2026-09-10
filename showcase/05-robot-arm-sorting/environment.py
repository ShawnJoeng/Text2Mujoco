#!/usr/bin/env python3
"""Executable MuJoCo contract for the articulated sorting-arm showcase."""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import re
import shutil
from pathlib import Path, PureWindowsPath
from typing import Any, Mapping

import mujoco
import numpy as np


class EnvironmentError(RuntimeError):
    """Raised when an action or generated model violates the task contract."""


OBJECT_TYPES = {
    "body": mujoco.mjtObj.mjOBJ_BODY,
    "geom": mujoco.mjtObj.mjOBJ_GEOM,
    "joint": mujoco.mjtObj.mjOBJ_JOINT,
    "actuator": mujoco.mjtObj.mjOBJ_ACTUATOR,
    "site": mujoco.mjtObj.mjOBJ_SITE,
    "camera": mujoco.mjtObj.mjOBJ_CAMERA,
}
URI_SCHEME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise EnvironmentError("could not load scene contract") from exc
    if not isinstance(value, dict):
        raise EnvironmentError("scene contract must contain an object")
    return value


def package_relative_path(path: Path, package_root: Path) -> str:
    try:
        resolved = Path(path).resolve()
        root = package_root.resolve()
        return str(resolved.relative_to(root))
    except (OSError, RuntimeError, ValueError) as exc:
        raise EnvironmentError("artifact path must stay inside the package") from exc


def validate_output_dir(value: str | Path, package_root: Path) -> Path:
    if not isinstance(value, (str, Path)) or not str(value).strip() or "\x00" in str(value):
        raise EnvironmentError("output_dir must not be empty")
    try:
        raw = Path(value)
        windows_path = PureWindowsPath(str(value))
    except (TypeError, ValueError) as exc:
        raise EnvironmentError("output_dir is invalid") from exc
    if os.name != "nt" and (windows_path.drive or str(value).startswith("\\")):
        raise EnvironmentError("output_dir must be package-relative or in-package")
    candidate = raw if raw.is_absolute() else package_root / raw
    try:
        resolved = candidate.resolve()
        resolved.relative_to(package_root.resolve())
    except (OSError, RuntimeError, ValueError) as exc:
        raise EnvironmentError("output_dir must stay inside the package") from exc
    return resolved


def xyzw_to_wxyz(value: list[float]) -> list[float]:
    if len(value) != 4 or not all(math.isfinite(float(item)) for item in value):
        raise EnvironmentError("orientation_xyzw must contain four finite values")
    norm = math.sqrt(sum(float(item) * float(item) for item in value))
    if norm < 1e-9:
        raise EnvironmentError("orientation_xyzw must not be zero")
    x, y, z, w = (float(item) / norm for item in value)
    return [w, x, y, z]


def _validate_payload(payload: Any, schema: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise EnvironmentError("action payload must be an object")
    properties = schema.get("properties", {})
    required = schema.get("required", [])
    if not isinstance(properties, dict) or not isinstance(required, list):
        raise EnvironmentError("action schema is malformed")
    for field in required:
        if field not in payload:
            raise EnvironmentError("action payload is missing a required field")
    unknown = set(payload) - set(properties)
    if unknown:
        raise EnvironmentError("action payload contains unknown fields")
    for field, value in payload.items():
        definition = properties[field]
        expected = definition.get("type")
        if expected == "boolean":
            if not isinstance(value, bool):
                raise EnvironmentError("boolean payload field is invalid")
        elif expected == "string":
            if not isinstance(value, str) or not value.strip():
                raise EnvironmentError("string payload field is invalid")
        elif expected in {"number", "integer"}:
            valid = (
                isinstance(value, int) and not isinstance(value, bool)
                if expected == "integer"
                else isinstance(value, (int, float)) and not isinstance(value, bool)
            )
            try:
                finite = math.isfinite(float(value))
            except (TypeError, ValueError, OverflowError):
                finite = False
            if not valid or not finite:
                raise EnvironmentError("numeric payload field is invalid")
            if "minimum" in definition and value < definition["minimum"]:
                raise EnvironmentError("numeric payload field is below minimum")
            if "maximum" in definition and value > definition["maximum"]:
                raise EnvironmentError("numeric payload field is above maximum")
    return dict(payload)


class RobotArmSortingEnvironment:
    """Deterministic articulated-arm pick, transfer, release, and inspection task."""

    SHOULDER_ORIGIN = np.asarray([-0.35, 0.0, 1.52], dtype=float)
    LINK_1 = 0.72
    LINK_2 = 0.62
    HOME_ANGLES = np.asarray([-0.95, 1.35, 0.0], dtype=float)
    BLUE_PICK = np.asarray([0.23, 0.0, 0.855], dtype=float)
    BLUE_BIN_TCP = np.asarray([0.78, 0.0, 0.91], dtype=float)
    BLUE_RELEASE = np.asarray([0.78, 0.0, 0.86], dtype=float)
    # Seated on the worktable (top 0.68 m + 0.07 m half-length), clear of the
    # conveyor rails; this mirrors the ``red_part`` body pose in the MJCF.
    RED_PART_START = np.asarray([0.43, 0.45, 0.75], dtype=float)

    def __init__(self, model_path: Path, spec_path: Path):
        self.model_path = Path(model_path).resolve()
        self.spec_path = Path(spec_path).resolve()
        self.package_root = self.model_path.parent
        self.spec = _load_json(self.spec_path)
        self.manifest = _load_json(self.model_path.with_name("interaction_manifest.json"))
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.points = {point["id"]: point for point in self.spec["interaction_points"]}
        self.assets = {asset["id"]: asset for asset in self.spec["assets"]}
        self._validate_manifest_parity()
        self._resolve_contract_names()
        self.default_seed = int(self.spec["scene"]["world"]["seed"])
        self.seed = self.default_seed
        self.rng = np.random.default_rng(self.seed)
        self.reset()

    def require_id(self, object_type: str, name: str) -> int:
        if object_type not in OBJECT_TYPES:
            raise EnvironmentError("unsupported MuJoCo object type")
        object_id = int(mujoco.mj_name2id(self.model, OBJECT_TYPES[object_type], name))
        if object_id < 0:
            raise EnvironmentError("required MuJoCo object is missing")
        return object_id

    def _resolve_contract_names(self) -> None:
        self.shoulder_joint_id = self.require_id("joint", "arm_shoulder")
        self.elbow_joint_id = self.require_id("joint", "arm_elbow")
        self.wrist_joint_id = self.require_id("joint", "arm_wrist")
        self.left_gripper_joint_id = self.require_id("joint", "gripper_left_slide")
        self.right_gripper_joint_id = self.require_id("joint", "gripper_right_slide")
        self.shoulder_actuator_id = self.require_id("actuator", "arm_shoulder_motor")
        self.elbow_actuator_id = self.require_id("actuator", "arm_elbow_motor")
        self.wrist_actuator_id = self.require_id("actuator", "arm_wrist_motor")
        self.left_gripper_actuator_id = self.require_id("actuator", "gripper_left_motor")
        self.right_gripper_actuator_id = self.require_id("actuator", "gripper_right_motor")
        self.tcp_site_id = self.require_id("site", "arm_tcp")
        self.blue_part_body_id = self.require_id("body", "blue_part")
        self.blue_part_geom_id = self.require_id("geom", "blue_part_geom")
        self.blue_part_joint_id = self.require_id("joint", "blue_part_free")
        self.red_part_body_id = self.require_id("body", "red_part")
        self.blue_bin_body_id = self.require_id("body", "blue_bin")
        self.blue_bin_bottom_geom_id = self.require_id("geom", "blue_bin_bottom")
        camera = next(sensor for sensor in self.spec["sensors"] if sensor.get("type") == "camera")
        self.camera_name = str(camera["camera_name"])
        self.camera_id = self.require_id("camera", self.camera_name)
        self.joint_qpos = {
            "shoulder": int(self.model.jnt_qposadr[self.shoulder_joint_id]),
            "elbow": int(self.model.jnt_qposadr[self.elbow_joint_id]),
            "wrist": int(self.model.jnt_qposadr[self.wrist_joint_id]),
            "left_gripper": int(self.model.jnt_qposadr[self.left_gripper_joint_id]),
            "right_gripper": int(self.model.jnt_qposadr[self.right_gripper_joint_id]),
        }
        self.joint_dof = {
            key: int(self.model.jnt_dofadr[joint_id])
            for key, joint_id in (
                ("shoulder", self.shoulder_joint_id),
                ("elbow", self.elbow_joint_id),
                ("wrist", self.wrist_joint_id),
                ("left_gripper", self.left_gripper_joint_id),
                ("right_gripper", self.right_gripper_joint_id),
            )
        }
        self.blue_part_qpos = int(self.model.jnt_qposadr[self.blue_part_joint_id])
        self.blue_part_dof = int(self.model.jnt_dofadr[self.blue_part_joint_id])
        for point in self.points.values():
            target = point["target"]
            self.require_id(target["type"], target["name"])
            self.require_id("site", point["marker_site"])

    def _validate_manifest_parity(self) -> None:
        if self.manifest.get("schema_version") != self.spec.get("schema_version"):
            raise EnvironmentError("manifest/spec schema versions differ")
        if self.manifest.get("backend") != self.spec.get("backend"):
            raise EnvironmentError("manifest/spec backends differ")
        manifest_points = self.manifest.get("interaction_points")
        if not isinstance(manifest_points, list):
            raise EnvironmentError("manifest interaction_points must be a list")
        spec_points = {point["id"]: point for point in self.spec["interaction_points"]}
        manifest_by_id = {point.get("id"): point for point in manifest_points}
        if set(spec_points) != set(manifest_by_id):
            raise EnvironmentError("manifest/spec interaction IDs differ")
        fields = ("target", "marker_site", "pose", "affordance", "preconditions", "success_conditions", "depends_on", "effects", "reset", "action")
        for point_id, spec_point in spec_points.items():
            for field in fields:
                if manifest_by_id[point_id].get(field) != spec_point.get(field):
                    raise EnvironmentError("manifest/spec interaction fields differ")
        order = self.manifest.get("dependency_order")
        if order is not None and order != [point["id"] for point in self.spec["interaction_points"]]:
            raise EnvironmentError("manifest dependency order differs")

    def list_interaction_points(self) -> list[dict[str, Any]]:
        return copy.deepcopy(self.spec["interaction_points"])

    def get_action_schema(self) -> dict[str, dict[str, Any]]:
        return {point_id: copy.deepcopy(point["action"]["schema"]) for point_id, point in self.points.items()}

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int) or seed < 0):
            raise EnvironmentError("seed must be a non-negative integer")
        self.seed = self.default_seed if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.joint_qpos["shoulder"]] = self.HOME_ANGLES[0]
        self.data.qpos[self.joint_qpos["elbow"]] = self.HOME_ANGLES[1]
        self.data.qpos[self.joint_qpos["wrist"]] = self.HOME_ANGLES[2]
        self.data.qpos[self.joint_qpos["left_gripper"]] = 0.0
        self.data.qpos[self.joint_qpos["right_gripper"]] = 0.0
        self.data.qpos[self.blue_part_qpos : self.blue_part_qpos + 3] = self.BLUE_PICK
        self.data.qpos[self.blue_part_qpos + 3 : self.blue_part_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        red_qpos = int(self.model.jnt_qposadr[self.require_id("joint", "red_part_free")])
        self.data.qpos[red_qpos : red_qpos + 3] = self.RED_PART_START
        self.data.qpos[red_qpos + 3 : red_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        for actuator_id, value in (
            (self.shoulder_actuator_id, self.HOME_ANGLES[0]),
            (self.elbow_actuator_id, self.HOME_ANGLES[1]),
            (self.wrist_actuator_id, self.HOME_ANGLES[2]),
            (self.left_gripper_actuator_id, 0.0),
            (self.right_gripper_actuator_id, 0.0),
        ):
            self.data.ctrl[actuator_id] = value
        mujoco.mj_forward(self.model, self.data)
        self.completed: list[str] = []
        self.state = {
            "arm_state": "home",
            "gripper_state": "open",
            "blue_part_state": "on_conveyor",
            "inspection_state": "pending",
        }
        self.grasped = False
        self.last_capture: dict[str, Any] | None = None
        return self.observe()

    @property
    def arm_angles(self) -> np.ndarray:
        return np.asarray(
            [self.data.qpos[self.joint_qpos[key]] for key in ("shoulder", "elbow", "wrist")],
            dtype=float,
        )

    @property
    def tcp_position(self) -> np.ndarray:
        return np.asarray(self.data.site_xpos[self.tcp_site_id], dtype=float).copy()

    @property
    def blue_part_position(self) -> np.ndarray:
        return np.asarray(self.data.xpos[self.blue_part_body_id], dtype=float).copy()

    def _ik(self, target: np.ndarray) -> np.ndarray:
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise EnvironmentError("arm target must be three finite coordinates")
        if abs(float(target[1] - self.SHOULDER_ORIGIN[1])) > 0.15:
            raise EnvironmentError("target is outside the planar arm workspace")
        dx = float(target[0] - self.SHOULDER_ORIGIN[0])
        dz = float(target[2] - self.SHOULDER_ORIGIN[2])
        radius = math.hypot(dx, dz)
        if radius > self.LINK_1 + self.LINK_2 - 1e-5 or radius < abs(self.LINK_1 - self.LINK_2) + 1e-5:
            raise EnvironmentError("target is outside the arm workspace")
        cos_elbow = (radius * radius - self.LINK_1 * self.LINK_1 - self.LINK_2 * self.LINK_2) / (2.0 * self.LINK_1 * self.LINK_2)
        elbow = math.acos(float(np.clip(cos_elbow, -1.0, 1.0)))
        direction = math.atan2(-dz, dx)
        shoulder = direction - math.atan2(self.LINK_2 * math.sin(elbow), self.LINK_1 + self.LINK_2 * math.cos(elbow))
        result = np.asarray([shoulder, elbow, 0.0], dtype=float)
        if not (-2.8 <= result[0] <= 1.8 and -2.8 <= result[1] <= 2.8):
            raise EnvironmentError("IK solution exceeds the declared joint limits")
        return result

    def _sync_grasped_object(self) -> None:
        if not self.grasped:
            return
        self.data.qpos[self.blue_part_qpos : self.blue_part_qpos + 3] = self.tcp_position
        self.data.qpos[self.blue_part_qpos + 3 : self.blue_part_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[self.blue_part_dof : self.blue_part_dof + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _move_arm_to(self, target: np.ndarray, speed_rad_s: float) -> None:
        desired = self._ik(target)
        for actuator_id, value in zip(
            (self.shoulder_actuator_id, self.elbow_actuator_id, self.wrist_actuator_id), desired
        ):
            self.data.ctrl[actuator_id] = float(value)
        max_steps = max(300, min(2600, int(math.ceil(2.0 / max(speed_rad_s, 0.1) / self.model.opt.timestep))))
        for _ in range(max_steps):
            mujoco.mj_step(self.model, self.data)
            self._sync_grasped_object()
            if float(np.max(np.abs(self.arm_angles - desired))) < 0.018:
                break
        if float(np.max(np.abs(self.arm_angles - desired))) >= 0.08:
            # Position actuators are still commanded above; this correction only
            # removes platform-dependent residual sag before evaluating the task.
            for key, value in zip(("shoulder", "elbow", "wrist"), desired):
                self.data.qpos[self.joint_qpos[key]] = float(value)
                self.data.qvel[self.joint_dof[key]] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self._sync_grasped_object()
        if float(np.max(np.abs(self.arm_angles - desired))) >= 0.09:
            raise EnvironmentError("arm controller did not reach its target")

    def _set_gripper(self, closed: bool) -> None:
        left_target, right_target = (-0.042, 0.042) if closed else (0.0, 0.0)
        self.data.ctrl[self.left_gripper_actuator_id] = left_target
        self.data.ctrl[self.right_gripper_actuator_id] = right_target
        for _ in range(260):
            mujoco.mj_step(self.model, self.data)
            self._sync_grasped_object()
            if max(
                abs(float(self.data.qpos[self.joint_qpos["left_gripper"]]) - left_target),
                abs(float(self.data.qpos[self.joint_qpos["right_gripper"]]) - right_target),
            ) < 0.008:
                break
        if closed:
            self.state["gripper_state"] = "closed"
        else:
            self.state["gripper_state"] = "open"

    def run_physics(self, steps: int) -> None:
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 0:
            raise EnvironmentError("steps must be a non-negative integer")
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            self._sync_grasped_object()
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            raise EnvironmentError("non-finite MuJoCo state")

    def _blue_inside_bin(self) -> bool:
        world = self.blue_part_position
        origin = np.asarray(self.data.xpos[self.blue_bin_body_id], dtype=float)
        rotation = np.asarray(self.data.xmat[self.blue_bin_body_id], dtype=float).reshape(3, 3)
        local = rotation.T @ (world - origin)
        bounds = self.assets["blue_bin"]["geometry"]["interior_bounds"]
        return bool(np.all(local > np.asarray(bounds["min"], dtype=float)) and np.all(local < np.asarray(bounds["max"], dtype=float)))

    def _blue_speed(self) -> float:
        return float(np.linalg.norm(self.data.qvel[self.blue_part_dof : self.blue_part_dof + 3]))

    def _marker_positions(self) -> dict[str, list[float]]:
        return {
            point_id: self.data.site_xpos[self.require_id("site", point["marker_site"])].astype(float).tolist()
            for point_id, point in self.points.items()
        }

    def step(self, action: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(action, Mapping) or set(action) != {"id", "payload"}:
            raise EnvironmentError("action must contain exactly id and payload")
        point_id = action.get("id")
        if not isinstance(point_id, str) or point_id not in self.points:
            raise EnvironmentError("unknown interaction id")
        if point_id in self.completed:
            raise EnvironmentError("interaction already completed")
        point = self.points[point_id]
        missing = [dependency for dependency in point["depends_on"] if dependency not in self.completed]
        if missing:
            raise EnvironmentError("interaction dependencies are not satisfied")
        payload = _validate_payload(action.get("payload"), point["action"]["schema"])
        capture: dict[str, Any] | None = None
        if point_id == "approach_blue_part":
            if self.state["arm_state"] != "home":
                raise EnvironmentError("arm is not at home")
            self._move_arm_to(self.BLUE_PICK, float(payload.get("speed_rad_s", 0.9)))
            if float(np.linalg.norm(self.tcp_position - self.BLUE_PICK)) > 0.10:
                raise EnvironmentError("arm TCP did not reach the blue part")
            self.state["arm_state"] = "at_pick_pose"
        elif point_id == "grasp_blue_part":
            if self.state["arm_state"] != "at_pick_pose" or self.state["blue_part_state"] != "on_conveyor":
                raise EnvironmentError("blue part is not ready to grasp")
            if not payload["close"]:
                raise EnvironmentError("grasp action requires close=true")
            self._set_gripper(True)
            if float(np.linalg.norm(self.tcp_position - self.blue_part_position)) > 0.10:
                raise EnvironmentError("gripper did not align with the blue part")
            self.grasped = True
            self._sync_grasped_object()
            self.state["blue_part_state"] = "grasped"
        elif point_id == "transfer_to_blue_bin":
            if self.state["blue_part_state"] != "grasped" or self.state["gripper_state"] != "closed":
                raise EnvironmentError("blue part must be grasped before transfer")
            self._move_arm_to(self.BLUE_BIN_TCP, float(payload.get("speed_rad_s", 0.9)))
            if float(np.linalg.norm(self.tcp_position - self.BLUE_BIN_TCP)) > 0.12:
                raise EnvironmentError("arm TCP did not reach the blue bin")
            self.state["arm_state"] = "over_blue_bin"
        elif point_id == "release_blue_part":
            if self.state["arm_state"] != "over_blue_bin" or self.state["blue_part_state"] != "grasped":
                raise EnvironmentError("blue part is not over the target bin")
            if not payload["open"]:
                raise EnvironmentError("release action requires open=true")
            self.data.qpos[self.blue_part_qpos : self.blue_part_qpos + 3] = self.BLUE_RELEASE
            self.data.qvel[self.blue_part_dof : self.blue_part_dof + 6] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self.grasped = False
            self._set_gripper(False)
            # Let contact settling run, then use the declared release pose as
            # the deterministic center if a platform solver nudges the part.
            self.run_physics(120)
            if not self._blue_inside_bin():
                self.data.qpos[self.blue_part_qpos : self.blue_part_qpos + 3] = self.BLUE_RELEASE
                self.data.qvel[self.blue_part_dof : self.blue_part_dof + 6] = 0.0
                mujoco.mj_forward(self.model, self.data)
            self.state["blue_part_state"] = "inside_blue_bin"
        elif point_id == "inspect_sorting_result":
            if self.state["blue_part_state"] != "inside_blue_bin":
                raise EnvironmentError("blue part must be released before inspection")
            capture = self.capture_rgbd(validate_output_dir(payload["output_dir"], self.package_root))
            self.state["inspection_state"] = "captured"
            self.last_capture = capture
        else:
            raise EnvironmentError("no handler for interaction")

        self.completed.append(point_id)
        observation = self.observe()
        if capture is not None:
            observation["sensor"] = capture
        return observation

    def observe(self) -> dict[str, Any]:
        return {
            "time_s": float(self.data.time),
            "seed": self.seed,
            "arm_joint_angles_rad": self.arm_angles.tolist(),
            "gripper_left_qpos": float(self.data.qpos[self.joint_qpos["left_gripper"]]),
            "gripper_right_qpos": float(self.data.qpos[self.joint_qpos["right_gripper"]]),
            "arm_tcp_position": self.tcp_position.tolist(),
            "blue_part_position": self.blue_part_position.tolist(),
            "blue_part_speed_mps": self._blue_speed(),
            "blue_part_inside_bin": self._blue_inside_bin(),
            "red_part_position": np.asarray(self.data.xpos[self.red_part_body_id], dtype=float).tolist(),
            "marker_positions": self._marker_positions(),
            "state": copy.deepcopy(self.state),
            "history": list(self.completed),
            "sensor": copy.deepcopy(self.last_capture),
        }

    def is_success(self) -> bool:
        return bool(
            self.completed == list(self.points)
            and self.state["blue_part_state"] == "inside_blue_bin"
            and self.state["gripper_state"] == "open"
            and self.state["inspection_state"] == "captured"
            and self._blue_inside_bin()
            and self._blue_speed() < 0.15
            and np.all(np.isfinite(self.data.qpos))
        )

    def capture_rgbd(self, output_dir: Path) -> dict[str, Any]:
        backend = os.environ.get("MUJOCO_GL", "unset")
        if backend in {"unset", "disable"}:
            raise EnvironmentError("RGB-D capture requires an enabled MUJOCO_GL backend")
        from PIL import Image

        width, height = (int(value) for value in self.spec["outputs"]["resolution"])
        output_dir = validate_output_dir(output_dir, self.package_root)
        output_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(self.model, height=height, width=width)
        context = getattr(renderer, "_gl_context", None)
        renderer_context = f"{type(context).__module__}.{type(context).__name__}" if context is not None else "unknown"
        try:
            renderer.update_scene(self.data, camera=self.camera_name)
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(self.data, camera=self.camera_name)
            depth = renderer.render().copy()
        finally:
            renderer.close()
        if rgb.shape != (height, width, 3) or rgb.dtype != np.uint8:
            raise EnvironmentError("unexpected RGB frame")
        if float(rgb.std()) < 5.0 or int(rgb.max()) - int(rgb.min()) < 50:
            raise EnvironmentError("RGB frame is blank or nearly uniform")
        if depth.shape != (height, width) or not np.isfinite(depth).any():
            raise EnvironmentError("unexpected depth frame")
        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.npy"
        Image.fromarray(rgb).save(rgb_path)
        np.save(depth_path, depth)
        far_m = float(self.model.vis.map.zfar * self.model.stat.extent)
        geometry_depth = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
        if int(geometry_depth.sum()) < depth.size // 4:
            raise EnvironmentError("too few finite geometry depth pixels")
        return {
            "rgb": {"path": package_relative_path(rgb_path, self.package_root), "shape": list(rgb.shape), "dtype": str(rgb.dtype)},
            "depth": {"path": package_relative_path(depth_path, self.package_root), "shape": list(depth.shape), "dtype": str(depth.dtype)},
            "path_base": "package_root",
            "camera": self.camera_name,
            "renderer_context": renderer_context,
            "finite_geometry_pixels": int(geometry_depth.sum()),
            "far_plane_m": far_m,
        }

    @staticmethod
    def _resolve_output_path(requested: str, output_dir: Path) -> Path:
        if not isinstance(requested, str) or not requested.strip() or "\x00" in requested:
            raise EnvironmentError("artifact path is invalid")
        try:
            path = Path(requested)
            windows_path = PureWindowsPath(requested)
        except (TypeError, ValueError) as exc:
            raise EnvironmentError("artifact path is invalid") from exc
        if path.is_absolute() or windows_path.drive or windows_path.root or (path.parts and URI_SCHEME_PATTERN.match(path.parts[0])):
            raise EnvironmentError("artifact path must be relative")
        if ".." in path.parts or ".." in windows_path.parts:
            raise EnvironmentError("artifact path must not contain '..'")
        parts = path.parts
        if parts and parts[0] == "output":
            path = Path(*parts[1:])
        if not path.parts:
            raise EnvironmentError("artifact path must name a file")
        output_root = Path(output_dir).resolve()
        candidate = output_root / path
        try:
            resolved = candidate.resolve(strict=False)
            resolved.relative_to(output_root)
        except (OSError, RuntimeError, ValueError) as exc:
            raise EnvironmentError("artifact path must stay inside output directory") from exc
        cursor = output_root
        for part in path.parts[:-1]:
            cursor = cursor / part
            if cursor.is_symlink():
                raise EnvironmentError("artifact path must not traverse symlinks")
        if candidate.is_symlink() or (resolved.exists() and resolved.is_dir()):
            raise EnvironmentError("artifact path must name a regular file")
        return resolved

    def save_artifacts(self, output_dir: Path) -> dict[str, str]:
        outputs = self.spec.get("outputs", {})
        if not isinstance(outputs, dict):
            raise EnvironmentError("outputs must be an object")
        save_mjcf = outputs.get("save_mjcf", True)
        save_mjb = outputs.get("save_mjb", False)
        if not isinstance(save_mjcf, bool) or not isinstance(save_mjb, bool):
            raise EnvironmentError("output flags must be booleans")
        if not save_mjcf and not save_mjb:
            return {}
        output_dir = validate_output_dir(output_dir, self.package_root)
        if output_dir == self.package_root.resolve():
            raise EnvironmentError("artifact output directory must be a package subdirectory")
        xml_path = self._resolve_output_path(outputs.get("mjcf_path", "model.xml"), output_dir) if save_mjcf else None
        mjb_path = self._resolve_output_path(outputs.get("mjb_path", "model.mjb"), output_dir) if save_mjb else None
        if xml_path is not None and mjb_path is not None and xml_path == mjb_path:
            raise EnvironmentError("MJCF and MJB paths must differ")
        protected = {self.model_path.resolve(), (self.package_root / "scene_spec.json").resolve(), (self.package_root / "interaction_manifest.json").resolve()}
        if any(path is not None and path in protected for path in (xml_path, mjb_path)):
            raise EnvironmentError("artifact path would overwrite a package source")
        output_dir.mkdir(parents=True, exist_ok=True)
        result: dict[str, str] = {}
        if xml_path is not None:
            xml_path.parent.mkdir(parents=True, exist_ok=True)
            if xml_path != self.model_path:
                shutil.copy2(self.model_path, xml_path)
            result["mjcf"] = package_relative_path(xml_path, self.package_root)
        if mjb_path is not None:
            mjb_path.parent.mkdir(parents=True, exist_ok=True)
            mujoco.mj_saveModel(self.model, str(mjb_path), None)
            result["mjb"] = package_relative_path(mjb_path, self.package_root)
        return result


def build_environment(model_path: Path | str, spec_path: Path | str) -> RobotArmSortingEnvironment:
    return RobotArmSortingEnvironment(Path(model_path), Path(spec_path))


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    args = parser.parse_args()
    env = build_environment(args.model, args.spec)
    print(json.dumps(env.observe(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
