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
    # Offset from the wrist origin to the tool centre point in the wrist frame:
    # 0.13 m along the wrist axis to the tool flange, then 0.26 m down to the
    # grasp centre between the finger pads. The TCP is not the wrist origin, so
    # a waypoint chosen for the wrist buries the gripper in whatever the wrist
    # is hovering over. The 0.26 m is what puts the carried part's top face
    # (0.26 - 0.07 = 0.19 m below the flange) 25 mm clear of the jaw sleeve.
    WRIST_TO_TCP = np.asarray([0.13, 0.0, -0.26], dtype=float)
    # Cartesian resolution of a commanded move, and the simulated seconds spent
    # on one segment at unit speed.
    PATH_STEP_M = 0.01
    SEGMENT_SECONDS = 0.028
    # Per-waypoint joint tracking error the path follower waits for. The whole
    # tool hangs below the wrist, so a standing error here is a palm that flies
    # lower than the commanded line.
    PATH_TRACK_RAD = 0.02
    # Finger pads mounted at +/-0.135 m with 0.025 m half-thickness close to an
    # inner face of 0.135 - 0.0415 - 0.025 = 0.0685 m, which is 1.5 mm *inside*
    # the 0.07 m part radius. The jaws bite on to the part instead of stopping
    # short of it, and a weld constraint then carries the load.
    GRIPPER_CLOSED = 0.0415
    # Tool pitch about the wrist axis. The wrist hinge used to be spent
    # cancelling the shoulder and elbow, which pinned the pitch to zero and made
    # the third joint decorative. Tipping the hand forward on the way in and
    # back on the way out is what lets the arm leave a part it has just set down
    # without dragging the whole tool column back through it.
    APPROACH_PITCH = -0.20
    WITHDRAW_PITCH = 0.24
    # Roll the jaws a quarter turn to lay the part into the bin in a chosen
    # orientation, then unwind before returning to the lane: rolled, the jaws
    # span x, and at the pick station both conveyor rollers are in that span.
    TRANSFER_ROLL = 1.5708
    ROLL_TRACK_RAD = 0.02
    RELEASE_SPEED_RAD_S = 0.9
    HOME_ANGLES = np.asarray([-0.95, 1.35, -0.40], dtype=float)
    BLUE_PICK = np.asarray([0.23, 0.0, 0.855], dtype=float)
    # Lift the payload's lowest surface (0.07 m below the TCP) to 1.03 m before
    # any travel over the bin, whose rim is at 0.975 m.
    PICK_HOVER = np.asarray([0.23, 0.0, 1.10], dtype=float)
    BLUE_BIN_HOVER = np.asarray([0.78, 0.0, 1.10], dtype=float)
    # Descend until the carried part is 1.5 mm above its resting height in the
    # bin (floor top 0.79 m + the part's 0.07 m half-length = 0.86 m), then let
    # go. The earlier 0.872 m left a 12 mm free fall, which landed hard enough to
    # squash 2.2 mm into the bin floor; 1.5 mm of standoff still means the weld
    # never presses the part into the floor, and the landing is 0.6 mm.
    BLUE_SET_DOWN = np.asarray([0.78, 0.0, 0.8615], dtype=float)
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

    def _require_weld(self, name: str) -> int:
        equality_id = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, name))
        if equality_id < 0:
            raise EnvironmentError("required equality constraint is missing")
        if int(self.model.eq_type[equality_id]) != int(mujoco.mjtEq.mjEQ_WELD):
            raise EnvironmentError("grasp constraint must be a weld")
        return equality_id

    def _resolve_contract_names(self) -> None:
        self.shoulder_joint_id = self.require_id("joint", "arm_shoulder")
        self.elbow_joint_id = self.require_id("joint", "arm_elbow")
        self.wrist_joint_id = self.require_id("joint", "arm_wrist")
        self.roll_joint_id = self.require_id("joint", "tool_roll")
        self.left_gripper_joint_id = self.require_id("joint", "gripper_left_slide")
        self.right_gripper_joint_id = self.require_id("joint", "gripper_right_slide")
        self.shoulder_actuator_id = self.require_id("actuator", "arm_shoulder_motor")
        self.elbow_actuator_id = self.require_id("actuator", "arm_elbow_motor")
        self.wrist_actuator_id = self.require_id("actuator", "arm_wrist_motor")
        self.roll_actuator_id = self.require_id("actuator", "tool_roll_motor")
        self.left_gripper_actuator_id = self.require_id("actuator", "gripper_left_motor")
        self.right_gripper_actuator_id = self.require_id("actuator", "gripper_right_motor")
        self.tcp_site_id = self.require_id("site", "arm_tcp")
        self.turret_body_id = self.require_id("body", "tool_turret")
        self.grasp_eq_id = self._require_weld("blue_part_grasp")
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
            "roll": int(self.model.jnt_qposadr[self.roll_joint_id]),
            "left_gripper": int(self.model.jnt_qposadr[self.left_gripper_joint_id]),
            "right_gripper": int(self.model.jnt_qposadr[self.right_gripper_joint_id]),
        }
        self.joint_dof = {
            key: int(self.model.jnt_dofadr[joint_id])
            for key, joint_id in (
                ("shoulder", self.shoulder_joint_id),
                ("elbow", self.elbow_joint_id),
                ("wrist", self.wrist_joint_id),
                ("roll", self.roll_joint_id),
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
        self.data.ctrl[:] = 0.0
        self.data.eq_active[self.grasp_eq_id] = 0
        self.data.qpos[self.joint_qpos["shoulder"]] = self.HOME_ANGLES[0]
        self.data.qpos[self.joint_qpos["elbow"]] = self.HOME_ANGLES[1]
        self.data.qpos[self.joint_qpos["wrist"]] = self.HOME_ANGLES[2]
        self.data.qpos[self.joint_qpos["roll"]] = 0.0
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
            (self.roll_actuator_id, 0.0),
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

    def _ik(self, target: np.ndarray, pitch: float = 0.0) -> np.ndarray:
        """Joint angles that put the tool centre point at ``target``.

        The two-link solve is for the wrist origin, so the tool offset is removed
        first - and that offset turns with the tool, hence the R_y(pitch) below.
        The wrist joint then absorbs the shoulder and elbow *plus* the requested
        pitch: with ``pitch = 0`` the pads point straight down, and any other
        value tips the whole hand by exactly that angle. The wrist used to be
        hard-wired to ``-(shoulder + elbow)``, which pinned the pitch to zero and
        left the third hinge with nothing of its own to do.
        """
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise EnvironmentError("arm target must be three finite coordinates")
        if not math.isfinite(pitch) or abs(float(pitch)) > 0.45:
            raise EnvironmentError("tool pitch is outside the declared envelope")
        if abs(float(target[1] - self.SHOULDER_ORIGIN[1])) > 0.15:
            raise EnvironmentError("target is outside the planar arm workspace")
        reach, _, drop = (float(value) for value in self.WRIST_TO_TCP)
        offset = np.asarray(
            [
                reach * math.cos(pitch) + drop * math.sin(pitch),
                0.0,
                -reach * math.sin(pitch) + drop * math.cos(pitch),
            ],
            dtype=float,
        )
        wrist_origin = target - offset
        dx = float(wrist_origin[0] - self.SHOULDER_ORIGIN[0])
        dz = float(wrist_origin[2] - self.SHOULDER_ORIGIN[2])
        radius = math.hypot(dx, dz)
        if radius > self.LINK_1 + self.LINK_2 - 1e-5 or radius < abs(self.LINK_1 - self.LINK_2) + 1e-5:
            raise EnvironmentError("target is outside the arm workspace")
        cos_elbow = (radius * radius - self.LINK_1 * self.LINK_1 - self.LINK_2 * self.LINK_2) / (2.0 * self.LINK_1 * self.LINK_2)
        elbow = math.acos(float(np.clip(cos_elbow, -1.0, 1.0)))
        direction = math.atan2(-dz, dx)
        shoulder = direction - math.atan2(self.LINK_2 * math.sin(elbow), self.LINK_1 + self.LINK_2 * math.cos(elbow))
        result = np.asarray([shoulder, elbow, -(shoulder + elbow) + float(pitch)], dtype=float)
        if not (-2.8 <= result[0] <= 1.8 and -2.8 <= result[1] <= 2.8 and -2.4 <= result[2] <= 2.4):
            raise EnvironmentError("IK solution exceeds the declared joint limits")
        return result

    def _engage_grasp(self) -> None:
        """Switch the weld on at the offset the jaws actually closed at.

        The part used to be carried by writing its free-joint qpos every step,
        which made a part lying on the belt indistinguishable from a part in the
        hand. Measuring the turret-to-part pose here and writing it into
        ``eq_data`` means the constraint engages already satisfied, so there is
        no jolt at the instant of the grasp and no invented translation either.
        """
        turret_position = np.asarray(self.data.xpos[self.turret_body_id], dtype=float)
        turret_frame = np.asarray(self.data.xmat[self.turret_body_id], dtype=float).reshape(3, 3)
        part_frame = np.asarray(self.data.xmat[self.blue_part_body_id], dtype=float).reshape(3, 3)
        relative = turret_frame.T @ (self.blue_part_position - turret_position)
        relative_quat = np.zeros(4, dtype=float)
        mujoco.mju_mat2Quat(relative_quat, np.ascontiguousarray(turret_frame.T @ part_frame).reshape(9))
        self.model.eq_data[self.grasp_eq_id, 3:6] = relative
        self.model.eq_data[self.grasp_eq_id, 6:10] = relative_quat
        self.data.eq_active[self.grasp_eq_id] = 1
        self.grasped = True

    def _release_grasp(self) -> None:
        """Let go. Nothing is written to the part; gravity takes it from here."""
        self.data.eq_active[self.grasp_eq_id] = 0
        self.grasped = False

    def _command(self, angles: np.ndarray) -> None:
        for actuator_id, value in zip(
            (self.shoulder_actuator_id, self.elbow_actuator_id, self.wrist_actuator_id), angles
        ):
            self.data.ctrl[actuator_id] = float(value)

    def _set_roll(self, angle: float) -> None:
        if not math.isfinite(angle) or abs(float(angle)) > 1.65:
            raise EnvironmentError("tool roll is outside the declared envelope")
        self.data.ctrl[self.roll_actuator_id] = float(angle)
        for _ in range(400):
            mujoco.mj_step(self.model, self.data)
            if abs(float(self.data.qpos[self.joint_qpos["roll"]]) - float(angle)) < self.ROLL_TRACK_RAD:
                break
        if abs(float(self.data.qpos[self.joint_qpos["roll"]]) - float(angle)) >= 0.05:
            raise EnvironmentError("tool roll servo did not reach its target")

    def _settle_at(self, target: np.ndarray, speed_rad_s: float, pitch: float = 0.0) -> None:
        desired = self._ik(target, pitch)
        self._command(desired)
        budget = max(200, min(1400, int(math.ceil(0.8 / max(speed_rad_s, 0.1) / self.model.opt.timestep))))
        for _ in range(budget):
            mujoco.mj_step(self.model, self.data)
            if float(np.max(np.abs(self.arm_angles - desired))) < 0.005:
                break
        # No qpos correction here. Writing the joints to their targets would hide
        # whatever the servos are actually lagging by, and with gravcomp carrying
        # the arm's own weight the residual is the payload term only: 2.04 Nm at
        # kp = 4000 is 0.51 mrad, so 0.03 rad is a genuine controller failure.
        if float(np.max(np.abs(self.arm_angles - desired))) >= 0.03:
            raise EnvironmentError("arm controller did not reach its target")

    def _follow_path(self, waypoints: list[np.ndarray], speed_rad_s: float, pitch: float = 0.0) -> None:
        """Drive the TCP along straight Cartesian segments through ``waypoints``.

        Commanding only the endpoint lets the joints interpolate freely, and the
        arc they choose dips well below both ends of the move: on this cell it
        dragged the held part down through the worktable while the start pose,
        the end pose, and the endpoint tolerance all stayed satisfied. Solving
        each intermediate point instead keeps the tool on the declared straight
        line, so a lift-traverse-lower path is the path the arm actually flies.

        Each segment is also held until the joints have caught up. Open-loop
        holds leave a standing tracking error, and because the whole tool hangs
        below the wrist that error shows up as the palm sitting lower than the
        commanded path.
        """
        pace = max(3, int(round(self.SEGMENT_SECONDS / max(speed_rad_s, 0.1) / self.model.opt.timestep)))
        for waypoint in waypoints:
            target = np.asarray(waypoint, dtype=float)
            self._ik(target, pitch)  # reject an unreachable waypoint before moving
            start = self.tcp_position
            segments = max(1, int(math.ceil(float(np.linalg.norm(target - start)) / self.PATH_STEP_M)))
            for index in range(1, segments + 1):
                desired = self._ik(start + (target - start) * (index / segments), pitch)
                self._command(desired)
                for held in range(4 * pace):
                    mujoco.mj_step(self.model, self.data)
                    if held >= pace and float(np.max(np.abs(self.arm_angles - desired))) < self.PATH_TRACK_RAD:
                        break
            self._settle_at(target, speed_rad_s, pitch)

    def _set_gripper(self, closed: bool) -> None:
        left_target, right_target = (
            (-self.GRIPPER_CLOSED, self.GRIPPER_CLOSED) if closed else (0.0, 0.0)
        )
        self.data.ctrl[self.left_gripper_actuator_id] = left_target
        self.data.ctrl[self.right_gripper_actuator_id] = right_target
        for _ in range(260):
            mujoco.mj_step(self.model, self.data)
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
            # Hover above the part first, then descend: the finger pads reach
            # 0.025 m below the TCP and would otherwise enter from the side. The
            # hand is tipped forward for the free-space leg and levelled again
            # before the descent, because a tilted jaw pair does not close square
            # on a cylinder.
            speed = float(payload.get("speed_rad_s", 0.9))
            self._follow_path([self.PICK_HOVER], speed, self.APPROACH_PITCH)
            self._settle_at(self.PICK_HOVER, speed, 0.0)
            self._follow_path([self.BLUE_PICK], speed, 0.0)
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
            self._engage_grasp()
            self.state["blue_part_state"] = "grasped"
        elif point_id == "transfer_to_blue_bin":
            if self.state["blue_part_state"] != "grasped" or self.state["gripper_state"] != "closed":
                raise EnvironmentError("blue part must be grasped before transfer")
            # Straight up clear of the conveyor rails, then across above the bin
            # rim. Travelling at pick height would drag the part into the wall.
            # The quarter turn waits until the tool is over the bin: rolled, the
            # jaws span x, and at the pick station both rollers are in that span.
            self._follow_path([self.PICK_HOVER, self.BLUE_BIN_HOVER], float(payload.get("speed_rad_s", 0.9)))
            self._set_roll(self.TRANSFER_ROLL)
            if float(np.linalg.norm(self.tcp_position - self.BLUE_BIN_HOVER)) > 0.12:
                raise EnvironmentError("arm TCP did not reach the blue bin")
            self.state["arm_state"] = "over_blue_bin"
        elif point_id == "release_blue_part":
            if self.state["arm_state"] != "over_blue_bin" or self.state["blue_part_state"] != "grasped":
                raise EnvironmentError("blue part is not over the target bin")
            if not payload["open"]:
                raise EnvironmentError("release action requires open=true")
            # Lower into the bin before letting go, so the part is set down
            # rather than dropped through the rim.
            self._follow_path([self.BLUE_SET_DOWN], self.RELEASE_SPEED_RAD_S)
            self._release_grasp()
            self._set_gripper(False)
            # The part is 1.5 mm above its resting height when the weld drops, so
            # it falls those 1.5 mm and settles. Nothing is written to its qpos:
            # if it ends up outside the bin that is a task failure to report, not
            # a pose to correct.
            self.run_physics(240)
            if not self._blue_inside_bin():
                raise EnvironmentError("released part did not come to rest inside the bin")
            # Withdraw so the evidence frame shows the sorted part, not a
            # gripper parked inside the bin. The roll unwinds on the way out
            # because the lane has no room for jaws spanning x, and the hand
            # tips back once it is clear of the rim so the open pads lift away
            # from the part instead of over it.
            self._follow_path([self.BLUE_BIN_HOVER], self.RELEASE_SPEED_RAD_S)
            self._set_roll(0.0)
            self._settle_at(self.BLUE_BIN_HOVER, self.RELEASE_SPEED_RAD_S, self.WITHDRAW_PITCH)
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
            "tool_roll_qpos": float(self.data.qpos[self.joint_qpos["roll"]]),
            "gripper_left_qpos": float(self.data.qpos[self.joint_qpos["left_gripper"]]),
            "gripper_right_qpos": float(self.data.qpos[self.joint_qpos["right_gripper"]]),
            "blue_part_held": bool(self.data.eq_active[self.grasp_eq_id]),
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
            and not bool(self.data.eq_active[self.grasp_eq_id])
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
