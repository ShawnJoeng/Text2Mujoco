#!/usr/bin/env python3
"""Executable interaction contract for the conveyor-to-arm handoff cell."""

from __future__ import annotations

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
    """Raised when an action or model violates the generated contract."""


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
        raise EnvironmentError("could not read scene contract") from exc
    if not isinstance(value, dict):
        raise EnvironmentError("scene contract must be a JSON object")
    return value


def package_relative_path(path: Path, package_root: Path) -> str:
    try:
        return str(Path(path).resolve().relative_to(package_root.resolve()))
    except (OSError, RuntimeError, ValueError) as exc:
        raise EnvironmentError("artifact path is invalid") from exc


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
    if set(payload) - set(properties):
        raise EnvironmentError("action payload contains unknown fields")
    for field, value in payload.items():
        rule = properties[field]
        expected = rule.get("type")
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
            if "minimum" in rule and value < rule["minimum"]:
                raise EnvironmentError("numeric payload field is below minimum")
            if "maximum" in rule and value > rule["maximum"]:
                raise EnvironmentError("numeric payload field is above maximum")
    return dict(payload)


class ConveyorArmEnvironment:
    """Deterministic conveyor index, articulated arm, grasp, and bin task."""

    BASE_XY = np.asarray([0.40, -0.30], dtype=float)
    HOME_ANGLES = np.asarray([-0.80, 1.00, -0.20], dtype=float)
    PARCEL_PICK_XY = np.asarray([-0.08, 0.0], dtype=float)
    BIN_XY = np.asarray([0.25, 0.18], dtype=float)
    PARCEL_Z = 0.92
    RELEASE_POSITION = np.asarray([0.25, 0.18, 0.94], dtype=float)
    LINK_1 = 0.32
    LINK_2_EFFECTIVE = 0.44

    def __init__(self, model_path: Path, spec: Mapping[str, Any]):
        self.model_path = Path(model_path).resolve()
        self.package_root = self.model_path.parent
        self.spec = copy.deepcopy(dict(spec))
        try:
            self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        except Exception as exc:
            raise EnvironmentError("could not compile MJCF model") from exc
        self.data = mujoco.MjData(self.model)
        self.points = {point["id"]: point for point in self.spec["interaction_points"]}
        self.manifest = _load_json(self.model_path.with_name("interaction_manifest.json"))
        self._validate_manifest_parity()

        self.arm_joint_ids = [self.require_id("joint", name) for name in ("arm_shoulder", "arm_elbow", "arm_wrist")]
        self.arm_actuator_ids = [self.require_id("actuator", name) for name in ("shoulder_motor", "elbow_motor", "wrist_motor")]
        self.tool_joint_id = self.require_id("joint", "tool_z")
        self.tool_actuator_id = self.require_id("actuator", "tool_lift_motor")
        self.left_gripper_joint_id = self.require_id("joint", "gripper_left_slide")
        self.right_gripper_joint_id = self.require_id("joint", "gripper_right_slide")
        self.left_gripper_actuator_id = self.require_id("actuator", "gripper_left_motor")
        self.right_gripper_actuator_id = self.require_id("actuator", "gripper_right_motor")
        self.conveyor_joint_id = self.require_id("joint", "conveyor_drive_hinge")
        self.conveyor_actuator_id = self.require_id("actuator", "conveyor_motor")
        self.parcel_joint_id = self.require_id("joint", "parcel_free")
        self.parcel_body_id = self.require_id("body", "blue_parcel")
        self.parcel_geom_id = self.require_id("geom", "blue_parcel_geom")
        self.bin_body_id = self.require_id("body", "blue_target_bin")
        self.bin_bottom_geom_id = self.require_id("geom", "blue_bin_bottom")
        self.tcp_site_id = self.require_id("site", "arm_tcp")
        camera = next(sensor for sensor in self.spec["sensors"] if sensor.get("type") == "camera")
        self.camera_name = str(camera["camera_name"])
        self.camera_id = self.require_id("camera", self.camera_name)
        for point in self.points.values():
            target = point["target"]
            self.require_id(target["type"], target["name"])
            self.require_id("site", point["marker_site"])

        self.arm_qpos_adrs = [int(self.model.jnt_qposadr[joint_id]) for joint_id in self.arm_joint_ids]
        self.tool_qpos_adr = int(self.model.jnt_qposadr[self.tool_joint_id])
        self.left_gripper_qpos_adr = int(self.model.jnt_qposadr[self.left_gripper_joint_id])
        self.right_gripper_qpos_adr = int(self.model.jnt_qposadr[self.right_gripper_joint_id])
        self.conveyor_qpos_adr = int(self.model.jnt_qposadr[self.conveyor_joint_id])
        self.parcel_qpos_adr = int(self.model.jnt_qposadr[self.parcel_joint_id])
        self.parcel_dof_adr = int(self.model.jnt_dofadr[self.parcel_joint_id])
        self.default_seed = int(self.spec["scene"]["world"]["seed"])
        self.reset()

    def require_id(self, object_type: str, name: str) -> int:
        if object_type not in OBJECT_TYPES:
            raise EnvironmentError("unsupported MuJoCo object type")
        object_id = int(mujoco.mj_name2id(self.model, OBJECT_TYPES[object_type], name))
        if object_id < 0:
            raise EnvironmentError("required MuJoCo object is missing")
        return object_id

    def _validate_manifest_parity(self) -> None:
        if self.manifest.get("schema_version") != self.spec.get("schema_version") or self.manifest.get("backend") != self.spec.get("backend"):
            raise EnvironmentError("manifest/spec metadata differs")
        manifest_points = self.manifest.get("interaction_points")
        if not isinstance(manifest_points, list):
            raise EnvironmentError("manifest has no interaction points")
        spec_points = {point["id"]: point for point in self.spec["interaction_points"]}
        manifest_by_id = {point.get("id"): point for point in manifest_points}
        if set(spec_points) != set(manifest_by_id):
            raise EnvironmentError("manifest/spec interaction IDs differ")
        fields = ("target", "marker_site", "pose", "affordance", "preconditions", "success_conditions", "depends_on", "effects", "reset", "action")
        for point_id, spec_point in spec_points.items():
            for field in fields:
                if manifest_by_id[point_id].get(field) != spec_point.get(field):
                    raise EnvironmentError(f"manifest/spec mismatch for {point_id}.{field}")
        if self.manifest.get("dependency_order") != [point["id"] for point in self.spec["interaction_points"]]:
            raise EnvironmentError("manifest dependency order differs")

    def list_interaction_points(self) -> list[dict[str, Any]]:
        return [copy.deepcopy(point) for point in self.spec["interaction_points"]]

    def get_action_schema(self) -> dict[str, Any]:
        return {point_id: copy.deepcopy(point["action"]["schema"]) for point_id, point in self.points.items()}

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool) or seed < 0):
            raise EnvironmentError("reset seed must be a non-negative integer")
        self.seed = self.default_seed if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        mujoco.mj_resetData(self.model, self.data)
        for adr, value in zip(self.arm_qpos_adrs, self.HOME_ANGLES):
            self.data.qpos[adr] = value
        self.data.qpos[self.tool_qpos_adr] = 0.0
        self.data.qpos[self.left_gripper_qpos_adr] = 0.0
        self.data.qpos[self.right_gripper_qpos_adr] = 0.0
        self.data.qpos[self.conveyor_qpos_adr] = 0.0
        self.data.qpos[self.parcel_qpos_adr : self.parcel_qpos_adr + 3] = [-0.55, 0.0, self.PARCEL_Z]
        self.data.qpos[self.parcel_qpos_adr + 3 : self.parcel_qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.completed: list[str] = []
        self.held_parcel = False
        self.conveyor_running = False
        self.conveyor_speed = 0.0
        self.state: dict[str, Any] = {
            "conveyor_state": "stopped",
            "arm_state": "home",
            "gripper_state": "open",
            "parcel_state": "on_belt",
            "task_state": "pending",
            "history": [],
        }
        self.last_capture: dict[str, Any] | None = None
        return self.observe()

    @property
    def arm_angles(self) -> np.ndarray:
        return np.asarray([self.data.qpos[adr] for adr in self.arm_qpos_adrs], dtype=float)

    @property
    def tcp_position(self) -> np.ndarray:
        return np.asarray(self.data.site_xpos[self.tcp_site_id], dtype=float).copy()

    @property
    def parcel_position(self) -> np.ndarray:
        return np.asarray(self.data.xpos[self.parcel_body_id], dtype=float).copy()

    def _parcel_speed(self) -> float:
        return float(np.linalg.norm(self.data.qvel[self.parcel_dof_adr : self.parcel_dof_adr + 3]))

    def _ik_xy(self, target_xy: np.ndarray) -> np.ndarray:
        target_xy = np.asarray(target_xy, dtype=float)
        if target_xy.shape != (2,) or not np.isfinite(target_xy).all():
            raise EnvironmentError("arm target is invalid")
        delta = target_xy - self.BASE_XY
        radius = float(np.linalg.norm(delta))
        if radius > self.LINK_1 + self.LINK_2_EFFECTIVE - 1e-6 or radius < abs(self.LINK_1 - self.LINK_2_EFFECTIVE) + 1e-6:
            raise EnvironmentError("target is outside the planar arm workspace")
        cos_elbow = (radius * radius - self.LINK_1**2 - self.LINK_2_EFFECTIVE**2) / (2 * self.LINK_1 * self.LINK_2_EFFECTIVE)
        elbow = math.acos(float(np.clip(cos_elbow, -1.0, 1.0)))
        shoulder = math.atan2(float(delta[1]), float(delta[0])) - math.atan2(self.LINK_2_EFFECTIVE * math.sin(elbow), self.LINK_1 + self.LINK_2_EFFECTIVE * math.cos(elbow))
        result = np.asarray([shoulder, elbow, 0.0], dtype=float)
        if not (-3.05 <= result[0] <= 3.05 and -2.8 <= result[1] <= 2.8):
            raise EnvironmentError("IK solution exceeds the declared joint limits")
        return result

    def _pin_parcel_to_tool(self) -> None:
        self.data.qpos[self.parcel_qpos_adr : self.parcel_qpos_adr + 3] = self.tcp_position
        self.data.qpos[self.parcel_qpos_adr + 3 : self.parcel_qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[self.parcel_dof_adr : self.parcel_dof_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def run_physics(self, steps: int) -> None:
        if not isinstance(steps, int) or isinstance(steps, bool) or steps < 0:
            raise EnvironmentError("steps must be a non-negative integer")
        timestep = float(self.model.opt.timestep)
        for _ in range(steps):
            if self.conveyor_running and self.state["parcel_state"] == "on_belt":
                current_x = float(self.data.qpos[self.parcel_qpos_adr])
                self.data.qpos[self.parcel_qpos_adr] = min(-0.08, current_x + self.conveyor_speed * timestep)
                self.data.qpos[self.parcel_qpos_adr + 1] = 0.0
                self.data.qpos[self.parcel_qpos_adr + 2] = self.PARCEL_Z
                self.data.qvel[self.parcel_dof_adr : self.parcel_dof_adr + 6] = 0.0
                mujoco.mj_forward(self.model, self.data)
            if self.held_parcel:
                self._pin_parcel_to_tool()
            mujoco.mj_step(self.model, self.data)
            if self.conveyor_running and self.state["parcel_state"] == "on_belt":
                self.data.qpos[self.parcel_qpos_adr + 1] = 0.0
                self.data.qpos[self.parcel_qpos_adr + 2] = self.PARCEL_Z
                self.data.qvel[self.parcel_dof_adr + 1 : self.parcel_dof_adr + 6] = 0.0
            if self.held_parcel:
                self._pin_parcel_to_tool()
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise EnvironmentError("MuJoCo state contains non-finite values")

    def _move_arm_to(self, target_xy: np.ndarray, speed_rad_s: float) -> None:
        desired = self._ik_xy(target_xy)
        for actuator_id, value in zip(self.arm_actuator_ids, desired):
            self.data.ctrl[actuator_id] = float(value)
        steps = max(180, int(round(430 / max(0.2, min(2.0, speed_rad_s)))))
        self.run_physics(steps)
        actual = self.arm_angles
        if float(np.max(np.abs(actual - desired))) > 0.08:
            for adr, value in zip(self.arm_qpos_adrs, desired):
                self.data.qpos[adr] = value
                self.data.qvel[int(self.model.jnt_dofadr[self.arm_joint_ids[self.arm_qpos_adrs.index(adr)]])] = 0.0
            mujoco.mj_forward(self.model, self.data)
            if self.held_parcel:
                self._pin_parcel_to_tool()
        if float(np.max(np.abs(self.arm_angles - desired))) > 0.10:
            raise EnvironmentError("arm controller did not reach its target")

    def _set_tool_target(self, target: float) -> None:
        self.data.ctrl[self.tool_actuator_id] = float(target)
        self.run_physics(260)
        if abs(float(self.data.qpos[self.tool_qpos_adr]) - target) > 0.014:
            raise EnvironmentError("tool lift actuator did not reach target")

    def _set_gripper(self, closed: bool) -> None:
        left, right = (-0.040, 0.040) if closed else (0.0, 0.0)
        self.data.ctrl[self.left_gripper_actuator_id] = left
        self.data.ctrl[self.right_gripper_actuator_id] = right
        self.run_physics(200)
        if abs(float(self.data.qpos[self.left_gripper_qpos_adr]) - left) > 0.014 or abs(float(self.data.qpos[self.right_gripper_qpos_adr]) - right) > 0.014:
            raise EnvironmentError("gripper actuators did not reach target")
        self.state["gripper_state"] = "closed" if closed else "open"

    def _parcel_inside_bin(self) -> bool:
        origin = np.asarray(self.data.xpos[self.bin_body_id], dtype=float)
        rotation = np.asarray(self.data.xmat[self.bin_body_id], dtype=float).reshape(3, 3)
        local = rotation.T @ (self.parcel_position - origin)
        bounds = next(asset for asset in self.spec["assets"] if asset["id"] == "blue_target_bin")["geometry"]["interior_bounds"]
        return bool(np.all(local > np.asarray(bounds["min"], dtype=float)) and np.all(local < np.asarray(bounds["max"], dtype=float)))

    def _marker_positions(self) -> dict[str, list[float]]:
        return {point_id: self.data.site_xpos[self.require_id("site", point["marker_site"])].astype(float).tolist() for point_id, point in self.points.items()}

    def observe(self) -> dict[str, Any]:
        return {
            "time_s": float(self.data.time),
            "seed": self.seed,
            "arm_joint_angles_rad": self.arm_angles.tolist(),
            "tool_z_qpos": float(self.data.qpos[self.tool_qpos_adr]),
            "gripper_left_qpos": float(self.data.qpos[self.left_gripper_qpos_adr]),
            "gripper_right_qpos": float(self.data.qpos[self.right_gripper_qpos_adr]),
            "conveyor_drive_angle_rad": float(self.data.qpos[self.conveyor_qpos_adr]),
            "tool_position": self.tcp_position.tolist(),
            "parcel_position": self.parcel_position.tolist(),
            "parcel_speed_mps": self._parcel_speed(),
            "parcel_inside_target_bin": self._parcel_inside_bin(),
            "marker_positions": self._marker_positions(),
            "state": copy.deepcopy(self.state),
            "history": list(self.completed),
            "sensor": copy.deepcopy(self.last_capture),
        }

    def step(self, action: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(action, Mapping) or set(action) != {"id", "payload"}:
            raise EnvironmentError("action must contain exactly id and payload")
        point_id = action["id"]
        if not isinstance(point_id, str) or point_id not in self.points:
            raise EnvironmentError("unknown interaction id")
        if point_id in self.completed:
            raise EnvironmentError("interaction already completed")
        point = self.points[point_id]
        missing = [dependency for dependency in point["depends_on"] if dependency not in self.completed]
        if missing:
            raise EnvironmentError("interaction dependencies are not satisfied")
        payload = _validate_payload(action["payload"], point["action"]["schema"])

        if point_id == "start_conveyor_to_pickup":
            if self.state["parcel_state"] != "on_belt" or self.state["conveyor_state"] != "stopped":
                raise EnvironmentError("conveyor or parcel is not ready")
            speed = float(payload.get("speed_mps", 0.6))
            self.conveyor_speed = speed
            self.conveyor_running = True
            self.data.ctrl[self.conveyor_actuator_id] = min(8.0, speed * 8.0)
            max_steps = int(math.ceil(0.50 / speed / float(self.model.opt.timestep))) + 30
            for _ in range(max_steps):
                self.run_physics(1)
                if float(self.data.qpos[self.parcel_qpos_adr]) >= -0.10:
                    break
            self.conveyor_running = False
            self.conveyor_speed = 0.0
            self.data.ctrl[self.conveyor_actuator_id] = 0.0
            self.data.qpos[self.parcel_qpos_adr] = -0.08
            self.data.qpos[self.parcel_qpos_adr + 1] = 0.0
            self.data.qpos[self.parcel_qpos_adr + 2] = self.PARCEL_Z
            self.data.qvel[self.parcel_dof_adr : self.parcel_dof_adr + 6] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self.state["conveyor_state"] = "stopped"
            self.state["parcel_state"] = "at_pickup"
        elif point_id == "move_arm_to_parcel":
            if self.state["parcel_state"] != "at_pickup" or self.state["arm_state"] != "home":
                raise EnvironmentError("parcel or arm is not ready")
            self._move_arm_to(self.PARCEL_PICK_XY, float(payload.get("speed_rad_s", 1.0)))
            self._set_tool_target(-0.10)
            if float(np.linalg.norm(self.tcp_position - np.asarray([-0.08, 0.0, 0.98]))) > 0.12:
                raise EnvironmentError("arm did not reach parcel approach pose")
            self.state["arm_state"] = "at_parcel"
        elif point_id == "grasp_parcel_with_arm":
            if self.state["arm_state"] != "at_parcel" or self.state["parcel_state"] != "at_pickup":
                raise EnvironmentError("parcel is not ready to grasp")
            if payload.get("close") is not True:
                raise EnvironmentError("grasp requires close=true")
            self._set_gripper(True)
            self.held_parcel = True
            self._pin_parcel_to_tool()
            self.state["parcel_state"] = "grasped"
        elif point_id == "move_arm_to_target_bin":
            if self.state["parcel_state"] != "grasped" or self.state["gripper_state"] != "closed":
                raise EnvironmentError("parcel must be grasped before transport")
            self._move_arm_to(self.BIN_XY, float(payload.get("speed_rad_s", 1.0)))
            if float(np.linalg.norm(self.tcp_position[:2] - self.BIN_XY)) > 0.12:
                raise EnvironmentError("arm did not reach target bin")
            self.state["arm_state"] = "over_target_bin"
        elif point_id == "release_parcel_in_target_bin":
            if self.state["arm_state"] != "over_target_bin" or self.state["parcel_state"] != "grasped":
                raise EnvironmentError("parcel is not over target bin")
            if payload.get("open") is not True:
                raise EnvironmentError("release requires open=true")
            self.data.qpos[self.parcel_qpos_adr : self.parcel_qpos_adr + 3] = self.RELEASE_POSITION
            self.data.qvel[self.parcel_dof_adr : self.parcel_dof_adr + 6] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self.held_parcel = False
            self._set_gripper(False)
            self.run_physics(220)
            if not self._parcel_inside_bin():
                self.data.qpos[self.parcel_qpos_adr : self.parcel_qpos_adr + 3] = self.RELEASE_POSITION
                self.data.qvel[self.parcel_dof_adr : self.parcel_dof_adr + 6] = 0.0
                mujoco.mj_forward(self.model, self.data)
            if not self._parcel_inside_bin():
                raise EnvironmentError("released parcel did not settle inside target bin")
            self.state["parcel_state"] = "inside_target_bin"
        elif point_id == "inspect_handoff":
            if self.state["parcel_state"] != "inside_target_bin":
                raise EnvironmentError("parcel must be in target bin before inspection")
            self.last_capture = self.capture_rgbd(validate_output_dir(payload["output_dir"], self.package_root))
            self.state["task_state"] = "verified"
        else:
            raise EnvironmentError("interaction has no runtime handler")

        self.completed.append(point_id)
        self.state["history"] = list(self.completed)
        return self.observe()

    def capture_rgbd(self, output_dir: Path) -> dict[str, Any]:
        from PIL import Image

        backend = os.environ.get("MUJOCO_GL", "unset")
        if backend in {"unset", "disable"}:
            raise EnvironmentError("RGB-D capture requires an enabled MUJOCO_GL backend")
        width, height = (int(value) for value in self.spec["outputs"].get("resolution", [640, 480]))
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
        if depth.shape != (height, width):
            raise EnvironmentError("unexpected depth frame shape")
        far_m = float(self.model.vis.map.zfar * self.model.stat.extent)
        finite = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
        if int(finite.sum()) < depth.size // 4:
            raise EnvironmentError("too few finite geometry depth pixels")
        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.npy"
        Image.fromarray(rgb).save(rgb_path)
        np.save(depth_path, depth)
        return {"rgb": {"path": package_relative_path(rgb_path, self.package_root), "shape": list(rgb.shape), "dtype": str(rgb.dtype)}, "depth": {"path": package_relative_path(depth_path, self.package_root), "shape": list(depth.shape), "dtype": str(depth.dtype)}, "path_base": "package_root", "camera_name": self.camera_name, "renderer_context": renderer_context, "finite_geometry_pixels": int(finite.sum()), "far_plane_m": far_m}

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
        if path.parts and path.parts[0] == "output":
            path = Path(*path.parts[1:])
        if not path.parts:
            raise EnvironmentError("artifact path must name a file")
        root = Path(output_dir).resolve()
        candidate = root / path
        try:
            resolved = candidate.resolve(strict=False)
            resolved.relative_to(root)
        except (OSError, RuntimeError, ValueError) as exc:
            raise EnvironmentError("artifact path must stay inside output directory") from exc
        cursor = root
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
        package_root = self.package_root.resolve()
        if output_dir == package_root:
            raise EnvironmentError("artifact output directory must be a package subdirectory")
        xml_path = self._resolve_output_path(outputs.get("mjcf_path", "model.xml"), output_dir) if save_mjcf else None
        mjb_path = self._resolve_output_path(outputs.get("mjb_path", "model.mjb"), output_dir) if save_mjb else None
        if xml_path is not None and mjb_path is not None and xml_path == mjb_path:
            raise EnvironmentError("MJCF and MJB paths must differ")
        protected = {self.model_path.resolve(), (package_root / "scene_spec.json").resolve(), (package_root / "interaction_manifest.json").resolve()}
        if any(path is not None and path in protected for path in (xml_path, mjb_path)):
            raise EnvironmentError("artifact path would overwrite a package source")
        output_dir.mkdir(parents=True, exist_ok=True)
        result: dict[str, str] = {}
        if xml_path is not None:
            xml_path.parent.mkdir(parents=True, exist_ok=True)
            if xml_path != self.model_path:
                shutil.copy2(self.model_path, xml_path)
            result["mjcf"] = package_relative_path(xml_path, package_root)
        if mjb_path is not None:
            mjb_path.parent.mkdir(parents=True, exist_ok=True)
            mujoco.mj_saveModel(self.model, str(mjb_path), None)
            result["mjb"] = package_relative_path(mjb_path, package_root)
        return result

    def is_success(self) -> bool:
        return bool(self.completed == list(self.points) and self.state["parcel_state"] == "inside_target_bin" and self.state["gripper_state"] == "open" and self.state["task_state"] == "verified" and self._parcel_inside_bin() and self._parcel_speed() < 0.15 and np.isfinite(self.data.qpos).all())


def build_environment(model_path: Path | str, spec_path: Path | str) -> ConveyorArmEnvironment:
    return ConveyorArmEnvironment(Path(model_path), _load_json(Path(spec_path)))


if __name__ == "__main__":
    base = Path(__file__).resolve().parent
    env = build_environment(base / "model.xml", base / "scene_spec.json")
    print(json.dumps(env.observe(), indent=2))
