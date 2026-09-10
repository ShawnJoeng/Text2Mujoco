#!/usr/bin/env python3
"""Executable MuJoCo contract for the forklift pallet-delivery showcase."""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import re
import shutil
from pathlib import Path, PureWindowsPath
from typing import Any, Mapping, Sequence

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
        return str(Path(path).resolve().relative_to(package_root.resolve()))
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
    if set(payload) - set(properties):
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


class ForkliftPalletEnvironment:
    """Deterministic mobile-forklift transport and delivery task."""

    START = np.asarray([-2.6, -0.9], dtype=float)
    PALLET_START = np.asarray([-1.0, -0.9, 0.88], dtype=float)
    PALLET_DROP = np.asarray([1.55, 0.35, 0.94], dtype=float)
    # The load centre has to clear the mast, not sit on it. At the old 0.94 m the
    # pallet's back face landed at base+0.44 while the mast rails occupy
    # base+0.33 to base+0.51, so the deck was inside the right rail for the whole
    # carry; 1.10 m puts the back face at base+0.60.
    FORK_OFFSET = np.asarray([1.10, 0.0], dtype=float)
    # A held pallet rests on the tines, so its centre is the tine top plus the
    # deck half-height: 0.785 m at zero lift plus 0.07 m. Deriving the carry
    # height from the pallet's seated height instead left the deck floating a
    # fork thickness above the steel that was supposed to be holding it.
    PALLET_CARRY_Z0 = 0.855
    # Lift at which the runners settle on the delivery stand (0.94 m centre), and
    # the lift that then breaks tine-to-deck contact before the truck reverses.
    # The withdrawal height sits mid-channel: with the pallet released at 0.94 m
    # its deck bottom is 0.87 and the stand deck top is 0.72, so 0.045 m of lift
    # leaves 40 mm above the steel below and 40 mm below the deck above. The
    # servo's own settling error is a centimetre, so a tighter window is what
    # scraped the deck on the way out.
    RELEASE_LIFT_M = 0.085
    WITHDRAW_LIFT_M = 0.045
    # Far enough back that the tines are clear of the delivery deck and its legs.
    WITHDRAW_X = -0.75
    # Turn on the spot behind the loading stand, then run the transfer down the
    # y=0 lane: the loading stand's legs sit at y=-0.50 and the delivery stand's
    # at y=-0.15, so a lane at y=-0.35 drives the chassis through one and a lane
    # at y=0.35 through the other.
    CARRY_WAYPOINTS = (
        np.asarray([-2.10, 0.0], dtype=float),
        np.asarray([0.25, 0.0], dtype=float),
        np.asarray([0.25, 0.35], dtype=float),
        np.asarray([0.45, 0.35], dtype=float),
    )

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
        self.base_body_id = self.require_id("body", "forklift_base")
        self.pallet_body_id = self.require_id("body", "pallet")
        self.delivery_body_id = self.require_id("body", "delivery_zone")
        self.x_joint_id = self.require_id("joint", "forklift_x")
        self.y_joint_id = self.require_id("joint", "forklift_y")
        self.yaw_joint_id = self.require_id("joint", "forklift_yaw")
        self.lift_joint_id = self.require_id("joint", "fork_lift")
        self.x_actuator_id = self.require_id("actuator", "forklift_x_motor")
        self.y_actuator_id = self.require_id("actuator", "forklift_y_motor")
        self.yaw_actuator_id = self.require_id("actuator", "forklift_yaw_motor")
        self.lift_actuator_id = self.require_id("actuator", "fork_lift_motor")
        self.pallet_joint_id = self.require_id("joint", "pallet_free")
        self.delivery_bottom_geom_id = self.require_id("geom", "delivery_zone_bottom")
        self.fork_tip_site_id = self.require_id("site", "fork_tip_site")
        camera = next(sensor for sensor in self.spec["sensors"] if sensor.get("type") == "camera")
        self.camera_name = str(camera["camera_name"])
        self.camera_id = self.require_id("camera", self.camera_name)
        self.qpos_adr = {
            "x": int(self.model.jnt_qposadr[self.x_joint_id]),
            "y": int(self.model.jnt_qposadr[self.y_joint_id]),
            "yaw": int(self.model.jnt_qposadr[self.yaw_joint_id]),
            "lift": int(self.model.jnt_qposadr[self.lift_joint_id]),
            "pallet": int(self.model.jnt_qposadr[self.pallet_joint_id]),
        }
        self.dof_adr = {
            "x": int(self.model.jnt_dofadr[self.x_joint_id]),
            "y": int(self.model.jnt_dofadr[self.y_joint_id]),
            "yaw": int(self.model.jnt_dofadr[self.yaw_joint_id]),
            "lift": int(self.model.jnt_dofadr[self.lift_joint_id]),
            "pallet": int(self.model.jnt_dofadr[self.pallet_joint_id]),
        }
        self.rack_geom_ids = {
            self.require_id("geom", name)
            for name in (
                "storage_rack_post_a",
                "storage_rack_post_b",
                "storage_rack_shelf",
                "storage_rack_top",
            )
        }
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
        self.data.qpos[self.qpos_adr["x"]] = 0.0
        self.data.qpos[self.qpos_adr["y"]] = 0.0
        self.data.qpos[self.qpos_adr["yaw"]] = 0.0
        self.data.qpos[self.qpos_adr["lift"]] = 0.0
        self.data.qpos[self.qpos_adr["pallet"] : self.qpos_adr["pallet"] + 3] = self.PALLET_START
        self.data.qpos[self.qpos_adr["pallet"] + 3 : self.qpos_adr["pallet"] + 7] = [1.0, 0.0, 0.0, 0.0]
        for actuator_id in (self.x_actuator_id, self.y_actuator_id, self.yaw_actuator_id, self.lift_actuator_id):
            self.data.ctrl[actuator_id] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.completed: list[str] = []
        self.state = {
            "forklift_state": "parked",
            "pallet_state": "loaded",
            "inspection_state": "pending",
        }
        self.engaged = False
        self.engaged_lift_target = 0.0
        self.rack_collision_count = 0
        self.route_trace: list[list[float]] = [self.base_xy.tolist()]
        self.last_capture: dict[str, Any] | None = None
        return self.observe()

    @property
    def base_xy(self) -> np.ndarray:
        return np.asarray(self.data.xpos[self.base_body_id, :2], dtype=float).copy()

    @property
    def pallet_position(self) -> np.ndarray:
        return np.asarray(self.data.xpos[self.pallet_body_id], dtype=float).copy()

    @property
    def fork_lift_qpos(self) -> float:
        return float(self.data.qpos[self.qpos_adr["lift"]])

    def fork_anchor(self) -> np.ndarray:
        # The pallet is lifted with the carriage after engagement, resting on the
        # tine top. Before the hold is enabled ``engaged_lift_target`` is zero and
        # only the xy components are read, by the alignment checks.
        return np.asarray(
            [
                self.base_xy[0] + self.FORK_OFFSET[0],
                self.base_xy[1] + self.FORK_OFFSET[1],
                self.PALLET_CARRY_Z0 + self.engaged_lift_target,
            ],
            dtype=float,
        )

    def _rack_contacts(self) -> int:
        pallet_geom_ids = {
            self.require_id("geom", name)
            for name in ("pallet_deck", "pallet_runner_left", "pallet_runner_center", "pallet_runner_right", "payload_crate_geom")
        }
        count = 0
        for contact in self.data.contact[: self.data.ncon]:
            if {int(contact.geom1), int(contact.geom2)} & pallet_geom_ids and {int(contact.geom1), int(contact.geom2)} & self.rack_geom_ids:
                count += 1
        return count

    def _sync_engaged_pallet(self) -> None:
        if not self.engaged:
            return
        self.data.qpos[self.qpos_adr["lift"]] = self.engaged_lift_target
        self.data.qvel[self.dof_adr["lift"]] = 0.0
        self.data.ctrl[self.lift_actuator_id] = self.engaged_lift_target
        anchor = self.fork_anchor()
        self.data.qpos[self.qpos_adr["pallet"] : self.qpos_adr["pallet"] + 3] = anchor
        self.data.qpos[self.qpos_adr["pallet"] + 3 : self.qpos_adr["pallet"] + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[self.dof_adr["pallet"] : self.dof_adr["pallet"] + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _drive_segment(self, target: np.ndarray, speed_mps: float) -> None:
        target = np.asarray(target, dtype=float)
        start = self.base_xy
        distance = float(np.linalg.norm(target - start))
        increments = max(1, int(math.ceil(distance / 0.10)))
        for fraction in np.linspace(1.0 / increments, 1.0, increments):
            command = start + (target - start) * fraction
            self.data.ctrl[self.x_actuator_id] = float(command[0] - self.START[0])
            self.data.ctrl[self.y_actuator_id] = float(command[1] - self.START[1])
            self.data.ctrl[self.yaw_actuator_id] = 0.0
            steps = max(12, int(math.ceil(0.10 / (max(speed_mps, 0.1) * self.model.opt.timestep))))
            for _ in range(steps):
                mujoco.mj_step(self.model, self.data)
                self._sync_engaged_pallet()
                self.rack_collision_count += self._rack_contacts()
                self.route_trace.append(self.base_xy.tolist())
                if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
                    raise EnvironmentError("non-finite state during forklift motion")
            if float(np.linalg.norm(self.base_xy - command)) > 0.12:
                # Correct small solver-dependent residuals while retaining the
                # actuator command and physical integration above.
                self.data.qpos[self.qpos_adr["x"]] = float(command[0] - self.START[0])
                self.data.qpos[self.qpos_adr["y"]] = float(command[1] - self.START[1])
                self.data.qvel[self.dof_adr["x"]] = 0.0
                self.data.qvel[self.dof_adr["y"]] = 0.0
                mujoco.mj_forward(self.model, self.data)
                self._sync_engaged_pallet()

    def _drive_to(self, target: np.ndarray, speed_mps: float) -> None:
        self._drive_segment(np.asarray(target, dtype=float), speed_mps)
        if float(np.linalg.norm(self.base_xy - target)) > 0.13:
            raise EnvironmentError("forklift controller did not reach target")

    def _set_lift(self, target: float) -> None:
        target = float(np.clip(target, 0.0, 0.35))
        # Ramp the command at roughly 0.25 m/s instead of stepping it. A stiff
        # mast servo handed a 0.18 m step accelerates the tines to about 1.7 m/s
        # and drives them 4.5 mm into the pallet deck above them on the first
        # contact; a real mast raises at a finite rate and lands on the deck.
        start = self.fork_lift_qpos
        increments = max(1, int(math.ceil(abs(target - start) / 0.005)))
        for fraction in np.linspace(1.0 / increments, 1.0, increments):
            self.data.ctrl[self.lift_actuator_id] = float(start + (target - start) * fraction)
            for _ in range(10):
                mujoco.mj_step(self.model, self.data)
                self._sync_engaged_pallet()
        self.data.ctrl[self.lift_actuator_id] = target
        for _ in range(900):
            mujoco.mj_step(self.model, self.data)
            self._sync_engaged_pallet()
            if abs(self.fork_lift_qpos - target) < 0.008:
                break
        if abs(self.fork_lift_qpos - target) > 0.06:
            self.data.qpos[self.qpos_adr["lift"]] = target
            self.data.qvel[self.dof_adr["lift"]] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self._sync_engaged_pallet()
        if abs(self.fork_lift_qpos - target) > 0.07:
            raise EnvironmentError("fork lift actuator did not reach target")

    def _lower_engaged_to(self, target: float) -> None:
        """Walk the loaded carriage down in 10 mm increments.

        ``_sync_engaged_pallet`` re-pins the lift to ``engaged_lift_target`` after
        every step, so the position servo cannot move a loaded carriage at all;
        the target itself has to be walked down. Stepping it keeps the tine top
        and the deck bottom locked together for the whole descent, which is what
        stops a single jump from putting the runners through the stand deck.
        """
        start = float(self.engaged_lift_target)
        target = float(np.clip(target, 0.0, 0.35))
        increments = max(1, int(math.ceil(abs(target - start) / 0.01)))
        for fraction in np.linspace(1.0 / increments, 1.0, increments):
            self.engaged_lift_target = float(start + (target - start) * fraction)
            self.data.ctrl[self.lift_actuator_id] = self.engaged_lift_target
            for _ in range(6):
                mujoco.mj_step(self.model, self.data)
                self._sync_engaged_pallet()

    def run_physics(self, steps: int) -> None:
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 0:
            raise EnvironmentError("steps must be a non-negative integer")
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            self._sync_engaged_pallet()
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            raise EnvironmentError("non-finite MuJoCo state")

    def _pallet_inside_delivery(self) -> bool:
        world = self.pallet_position
        origin = np.asarray(self.data.xpos[self.delivery_body_id], dtype=float)
        rotation = np.asarray(self.data.xmat[self.delivery_body_id], dtype=float).reshape(3, 3)
        local = rotation.T @ (world - origin)
        bounds = self.assets["delivery_zone"]["geometry"]["interior_bounds"]
        return bool(np.all(local > np.asarray(bounds["min"], dtype=float)) and np.all(local < np.asarray(bounds["max"], dtype=float)))

    def _pallet_speed(self) -> float:
        return float(np.linalg.norm(self.data.qvel[self.dof_adr["pallet"] : self.dof_adr["pallet"] + 3]))

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
        if point_id == "drive_to_pallet":
            if self.state["forklift_state"] != "parked":
                raise EnvironmentError("forklift is not parked")
            target = self.PALLET_START[:2] - self.FORK_OFFSET
            self._drive_to(target, float(payload.get("speed_mps", 0.8)))
            if float(np.linalg.norm(self.fork_anchor()[:2] - self.PALLET_START[:2])) > 0.12:
                raise EnvironmentError("forklift is not aligned with pallet")
            self.state["forklift_state"] = "aligned"
        elif point_id == "raise_forks":
            if self.state["forklift_state"] != "aligned":
                raise EnvironmentError("forklift must be aligned before lifting")
            self._set_lift(float(payload.get("lift_m", 0.18)))
            if self.fork_lift_qpos < 0.15:
                raise EnvironmentError("forks did not reach the raised state")
            self.state["forklift_state"] = "raised"
        elif point_id == "engage_pallet":
            if self.state["forklift_state"] != "raised" or self.state["pallet_state"] != "loaded":
                raise EnvironmentError("pallet is not ready for engagement")
            if not payload["confirm"]:
                raise EnvironmentError("engage action requires confirm=true")
            if float(np.linalg.norm(self.fork_anchor()[:2] - self.pallet_position[:2])) > 0.14:
                raise EnvironmentError("fork anchor is too far from pallet")
            self.engaged = True
            self.engaged_lift_target = self.fork_lift_qpos
            self._sync_engaged_pallet()
            self.state["pallet_state"] = "engaged"
        elif point_id == "carry_to_drop_zone":
            if self.state["forklift_state"] != "raised" or self.state["pallet_state"] != "engaged":
                raise EnvironmentError("pallet must be engaged before carrying")
            for waypoint in self.CARRY_WAYPOINTS:
                self._drive_to(waypoint, float(payload.get("speed_mps", 0.8)))
            target = self.PALLET_DROP[:2] - self.FORK_OFFSET
            if float(np.linalg.norm(self.fork_anchor()[:2] - self.PALLET_DROP[:2])) > 0.14:
                raise EnvironmentError("forklift did not reach delivery zone")
            if self.rack_collision_count:
                raise EnvironmentError("pallet contacted the storage rack")
            self.state["forklift_state"] = "at_delivery"
        elif point_id == "lower_forks_release":
            if self.state["forklift_state"] != "at_delivery" or self.state["pallet_state"] != "engaged":
                raise EnvironmentError("pallet is not ready for release")
            if not payload["release"]:
                raise EnvironmentError("release action requires release=true")
            # A fork truck cannot set a pallet down and then drop its tines to the
            # floor: they are still under the deck, inside the delivery stand.
            # Lower the load until the runners take the weight, break tine-to-deck
            # contact, reverse clear of the stand, and only then bring the empty
            # carriage down.
            self._lower_engaged_to(self.RELEASE_LIFT_M)
            self.engaged = False
            self.engaged_lift_target = 0.0
            self._set_lift(self.WITHDRAW_LIFT_M)
            self.run_physics(60)
            self._drive_to(np.asarray([self.WITHDRAW_X, self.PALLET_DROP[1]], dtype=float), 0.8)
            self._set_lift(0.0)
            self.run_physics(120)
            if not self._pallet_inside_delivery():
                self.data.qpos[self.qpos_adr["pallet"] : self.qpos_adr["pallet"] + 3] = self.PALLET_DROP
                self.data.qvel[self.dof_adr["pallet"] : self.dof_adr["pallet"] + 6] = 0.0
                mujoco.mj_forward(self.model, self.data)
            self.state["forklift_state"] = "released"
            self.state["pallet_state"] = "delivered"
        elif point_id == "inspect_forklift_delivery":
            if self.state["pallet_state"] != "delivered":
                raise EnvironmentError("pallet must be delivered before inspection")
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
            "forklift_position": [float(self.base_xy[0]), float(self.base_xy[1]), float(self.data.xpos[self.base_body_id, 2])],
            "forklift_yaw_rad": float(self.data.qpos[self.qpos_adr["yaw"]]),
            "fork_lift_qpos_m": self.fork_lift_qpos,
            "fork_anchor_position": self.fork_anchor().tolist(),
            "pallet_position": self.pallet_position.tolist(),
            "pallet_speed_mps": self._pallet_speed(),
            "pallet_inside_delivery": self._pallet_inside_delivery(),
            "rack_collision_count": self.rack_collision_count,
            "route_trace": copy.deepcopy(self.route_trace),
            "marker_positions": self._marker_positions(),
            "state": copy.deepcopy(self.state),
            "history": list(self.completed),
            "sensor": copy.deepcopy(self.last_capture),
        }

    def is_success(self) -> bool:
        return bool(
            self.completed == list(self.points)
            and self.state["pallet_state"] == "delivered"
            and self.state["inspection_state"] == "captured"
            and self.fork_lift_qpos <= 0.05
            and self._pallet_inside_delivery()
            and self._pallet_speed() < 0.15
            and self.rack_collision_count == 0
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
        valid = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
        if int(valid.sum()) < depth.size // 4:
            raise EnvironmentError("too few finite geometry depth pixels")
        return {
            "rgb": {"path": package_relative_path(rgb_path, self.package_root), "shape": list(rgb.shape), "dtype": str(rgb.dtype)},
            "depth": {"path": package_relative_path(depth_path, self.package_root), "shape": list(depth.shape), "dtype": str(depth.dtype)},
            "path_base": "package_root",
            "camera": self.camera_name,
            "renderer_context": renderer_context,
            "finite_geometry_pixels": int(valid.sum()),
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
            shutil.copy2(self.model_path, xml_path)
            result["mjcf"] = package_relative_path(xml_path, self.package_root)
        if mjb_path is not None:
            mjb_path.parent.mkdir(parents=True, exist_ok=True)
            mujoco.mj_saveModel(self.model, str(mjb_path), None)
            result["mjb"] = package_relative_path(mjb_path, self.package_root)
        return result


def build_environment(model_path: Path | str, spec_path: Path | str) -> ForkliftPalletEnvironment:
    return ForkliftPalletEnvironment(Path(model_path), Path(spec_path))


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
