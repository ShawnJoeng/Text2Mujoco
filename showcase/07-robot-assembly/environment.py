#!/usr/bin/env python3
"""Executable interaction contract for the robot peg-insertion assembly cell."""

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
        resolved = Path(path).resolve()
        root = package_root.resolve()
        return str(resolved.relative_to(root))
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
        raise EnvironmentError("output_dir must be package-relative or an in-package POSIX path")
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
    norm = math.sqrt(sum(float(item) ** 2 for item in value))
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
            raise EnvironmentError(f"action payload is missing required field: {field}")
    unknown = sorted(set(payload) - set(properties))
    if unknown:
        raise EnvironmentError("action payload has unknown fields")
    for field, value in payload.items():
        rule = properties[field]
        expected = rule.get("type")
        if expected == "boolean":
            if not isinstance(value, bool):
                raise EnvironmentError(f"payload.{field} must be boolean")
        elif expected == "string":
            if not isinstance(value, str) or not value.strip():
                raise EnvironmentError(f"payload.{field} must be a non-empty string")
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
                raise EnvironmentError(f"payload.{field} must be a finite {expected}")
            if "minimum" in rule and value < rule["minimum"]:
                raise EnvironmentError(f"payload.{field} is below minimum")
            if "maximum" in rule and value > rule["maximum"]:
                raise EnvironmentError(f"payload.{field} is above maximum")
    return dict(payload)


class RobotAssemblyEnvironment:
    """Deterministic planar arm, gripper, peg, and insertion-fixture task."""

    HOME = np.asarray([-0.80, 1.10, -0.30], dtype=float)
    AT_PEG = np.asarray([0.0, 0.0, 0.0], dtype=float)
    AT_SOCKET = np.asarray([0.20, 0.30, -0.40], dtype=float)
    SOCKET_POSITION = np.asarray([0.17, 0.21, 0.94], dtype=float)

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

        self.arm_joint_ids = [
            self.require_id("joint", name)
            for name in ("arm_shoulder", "arm_elbow", "arm_wrist")
        ]
        self.arm_actuator_ids = [
            self.require_id("actuator", name)
            for name in ("shoulder_motor", "elbow_motor", "wrist_motor")
        ]
        self.tool_joint_id = self.require_id("joint", "tool_z")
        self.tool_actuator_id = self.require_id("actuator", "tool_lift_motor")
        self.gripper_joint_id = self.require_id("joint", "gripper_slide")
        self.gripper_actuator_id = self.require_id("actuator", "gripper_motor")
        self.peg_joint_id = self.require_id("joint", "peg_free")
        self.peg_body_id = self.require_id("body", "red_peg")
        self.peg_geom_id = self.require_id("geom", "red_peg_geom")
        self.socket_floor_geom_id = self.require_id("geom", "socket_floor_geom")
        self.tool_site_id = self.require_id("site", "tool_center_site")
        camera = next(sensor for sensor in self.spec["sensors"] if sensor.get("type") == "camera")
        self.camera_name = str(camera["camera_name"])
        self.camera_id = self.require_id("camera", self.camera_name)
        for point in self.points.values():
            target = point["target"]
            self.require_id(target["type"], target["name"])
            self.require_id("site", point["marker_site"])

        self.arm_qpos_adrs = [int(self.model.jnt_qposadr[joint_id]) for joint_id in self.arm_joint_ids]
        self.tool_qpos_adr = int(self.model.jnt_qposadr[self.tool_joint_id])
        self.gripper_qpos_adr = int(self.model.jnt_qposadr[self.gripper_joint_id])
        self.peg_qpos_adr = int(self.model.jnt_qposadr[self.peg_joint_id])
        self.peg_dof_adr = int(self.model.jnt_dofadr[self.peg_joint_id])
        self.reset()

    def require_id(self, object_type: str, name: str) -> int:
        if object_type not in OBJECT_TYPES:
            raise EnvironmentError("unsupported MuJoCo object type")
        object_id = int(mujoco.mj_name2id(self.model, OBJECT_TYPES[object_type], name))
        if object_id < 0:
            raise EnvironmentError("required MuJoCo object is missing")
        return object_id

    def _validate_manifest_parity(self) -> None:
        if self.manifest.get("schema_version") != self.spec.get("schema_version"):
            raise EnvironmentError("manifest/spec schema versions differ")
        if self.manifest.get("backend") != self.spec.get("backend"):
            raise EnvironmentError("manifest/spec backends differ")
        manifest_points = self.manifest.get("interaction_points")
        if not isinstance(manifest_points, list):
            raise EnvironmentError("interaction manifest has no interaction points")
        spec_points = {point["id"]: point for point in self.spec["interaction_points"]}
        manifest_by_id = {point.get("id"): point for point in manifest_points}
        if set(spec_points) != set(manifest_by_id):
            raise EnvironmentError("manifest/spec interaction IDs differ")
        fields = ("target", "marker_site", "pose", "affordance", "preconditions", "success_conditions", "depends_on", "effects", "reset")
        for point_id, spec_point in spec_points.items():
            manifest_point = manifest_by_id[point_id]
            for field in fields:
                if manifest_point.get(field) != spec_point.get(field):
                    raise EnvironmentError(f"manifest/spec mismatch for {point_id}.{field}")
            if manifest_point.get("action") != spec_point.get("action"):
                raise EnvironmentError(f"manifest/spec mismatch for {point_id}.action")
        order = self.manifest.get("dependency_order")
        if order != [point["id"] for point in self.spec["interaction_points"]]:
            raise EnvironmentError("manifest dependency order differs")

    def list_interaction_points(self) -> list[dict[str, Any]]:
        return [copy.deepcopy(point) for point in self.spec["interaction_points"]]

    def get_action_schema(self) -> dict[str, Any]:
        return {point_id: copy.deepcopy(point["action"]["schema"]) for point_id, point in self.points.items()}

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool) or seed < 0):
            raise EnvironmentError("reset seed must be a non-negative integer")
        self.seed = int(self.spec["scene"]["world"]["seed"] if seed is None else seed)
        self.rng = np.random.default_rng(self.seed)
        mujoco.mj_resetData(self.model, self.data)
        for adr, value in zip(self.arm_qpos_adrs, self.HOME):
            self.data.qpos[adr] = value
        self.data.qpos[self.tool_qpos_adr] = 0.0
        self.data.qpos[self.gripper_qpos_adr] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.completed: list[str] = []
        self.held_peg = False
        self.state: dict[str, Any] = {
            "arm_state": "home",
            "gripper_state": "open",
            "tool_state": "raised",
            "peg_state": "free",
            "task_state": "pending",
            "history": [],
        }
        self.last_capture: dict[str, Any] | None = None
        return self.observe()

    def run_physics(self, steps: int) -> None:
        if not isinstance(steps, int) or isinstance(steps, bool) or steps < 0:
            raise EnvironmentError("steps must be a non-negative integer")
        for _ in range(steps):
            if self.held_peg:
                self._pin_peg_to_tool()
            mujoco.mj_step(self.model, self.data)
        if self.held_peg:
            self._pin_peg_to_tool()
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise EnvironmentError("MuJoCo state contains non-finite values")

    def _pin_peg_to_tool(self) -> None:
        position = np.asarray(self.data.site_xpos[self.tool_site_id], dtype=float)
        self.data.qpos[self.peg_qpos_adr : self.peg_qpos_adr + 3] = position
        self.data.qpos[self.peg_qpos_adr + 3 : self.peg_qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[self.peg_dof_adr : self.peg_dof_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _set_arm_targets(self, target: np.ndarray, speed_rad_s: float) -> None:
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise EnvironmentError("arm target is invalid")
        for actuator_id, value in zip(self.arm_actuator_ids, target):
            self.data.ctrl[actuator_id] = float(value)
        # Keep the trajectory long enough to expose intermediate arm states to
        # dense capture while remaining deterministic across machines.
        steps = max(120, int(round(420 / max(0.2, min(2.0, speed_rad_s)))))
        self.run_physics(steps)
        actual = np.asarray([self.data.qpos[adr] for adr in self.arm_qpos_adrs], dtype=float)
        if float(np.max(np.abs(actual - target))) > 0.06:
            raise EnvironmentError("arm actuators did not reach their target")

    def _set_tool_target(self, target: float) -> None:
        self.data.ctrl[self.tool_actuator_id] = float(target)
        self.run_physics(260)
        if abs(float(self.data.qpos[self.tool_qpos_adr]) - target) > 0.012:
            raise EnvironmentError("tool lift actuator did not reach its target")

    def _set_gripper_target(self, target: float) -> None:
        self.data.ctrl[self.gripper_actuator_id] = float(target)
        self.run_physics(180)
        if abs(float(self.data.qpos[self.gripper_qpos_adr]) - target) > 0.012:
            raise EnvironmentError("gripper actuator did not reach its target")

    def _peg_position(self) -> np.ndarray:
        return np.asarray(self.data.xpos[self.peg_body_id], dtype=float).copy()

    def _peg_speed(self) -> float:
        return float(np.linalg.norm(self.data.qvel[self.peg_dof_adr : self.peg_dof_adr + 3]))

    def _tool_position(self) -> np.ndarray:
        return np.asarray(self.data.site_xpos[self.tool_site_id], dtype=float).copy()

    def _marker_positions(self) -> dict[str, list[float]]:
        result: dict[str, list[float]] = {}
        for point in self.spec["interaction_points"]:
            site_id = self.require_id("site", point["marker_site"])
            result[point["id"]] = self.data.site_xpos[site_id].astype(float).tolist()
        return result

    def observe(self) -> dict[str, Any]:
        return {
            "time_s": float(self.data.time),
            "seed": self.seed,
            "arm_joint_qpos": [float(self.data.qpos[adr]) for adr in self.arm_qpos_adrs],
            "tool_z_qpos": float(self.data.qpos[self.tool_qpos_adr]),
            "gripper_qpos": float(self.data.qpos[self.gripper_qpos_adr]),
            "tool_position": self._tool_position().tolist(),
            "peg_position": self._peg_position().tolist(),
            "peg_speed_mps": self._peg_speed(),
            "peg_socket_error_m": float(np.linalg.norm(self._peg_position() - self.SOCKET_POSITION)),
            "marker_positions": self._marker_positions(),
            "state": copy.deepcopy(self.state),
            "history": list(self.completed),
            "sensor": copy.deepcopy(self.last_capture),
        }

    def _require_dependencies(self, point: Mapping[str, Any]) -> None:
        missing = [dependency for dependency in point["depends_on"] if dependency not in self.completed]
        if missing:
            raise EnvironmentError("unmet interaction dependencies")

    def step(self, action: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(action, Mapping) or set(action) != {"id", "payload"}:
            raise EnvironmentError("action must contain exactly id and payload")
        point_id = action["id"]
        if not isinstance(point_id, str) or point_id not in self.points:
            raise EnvironmentError("unknown interaction id")
        if point_id in self.completed:
            raise EnvironmentError("interaction already completed")
        point = self.points[point_id]
        self._require_dependencies(point)
        payload = _validate_payload(action["payload"], point["action"]["schema"])

        if point_id == "move_arm_to_peg":
            if self.state["arm_state"] != "home":
                raise EnvironmentError("arm is not at home")
            self._set_arm_targets(self.AT_PEG, float(payload.get("speed_rad_s", 1.0)))
            self.state["arm_state"] = "at_peg"
        elif point_id == "grasp_peg_with_arm":
            if self.state["arm_state"] != "at_peg" or self.state["peg_state"] != "free":
                raise EnvironmentError("arm must be at peg and peg must be free")
            if payload.get("close") is not True:
                raise EnvironmentError("grasp requires close=true")
            self._set_tool_target(-0.035)
            self._set_gripper_target(-0.040)
            # The attachment is a documented task-level grasp abstraction. It
            # is updated at every physics step, so subsequent arm motion and
            # the physical peg share one deterministic state trajectory.
            self.held_peg = True
            self._pin_peg_to_tool()
            self.state["gripper_state"] = "closed"
            self.state["peg_state"] = "grasped"
        elif point_id == "move_arm_to_socket":
            if self.state["arm_state"] != "at_peg" or self.state["peg_state"] != "grasped":
                raise EnvironmentError("peg must be grasped before transport")
            self._set_arm_targets(self.AT_SOCKET, float(payload.get("speed_rad_s", 1.0)))
            self.state["arm_state"] = "at_socket"
        elif point_id == "insert_peg_into_socket":
            if self.state["arm_state"] != "at_socket" or self.state["peg_state"] != "grasped":
                raise EnvironmentError("arm must be at socket with a grasped peg")
            depth = float(payload.get("depth_m", 0.08))
            self._set_tool_target(-depth)
            self.state["tool_state"] = "lowered"
            if float(self.data.qpos[self.tool_qpos_adr]) > -0.06:
                raise EnvironmentError("tool did not lower into insertion position")
            self.state["peg_state"] = "inserted"
        elif point_id == "release_assembled_peg":
            if self.state["peg_state"] != "inserted" or self.state["tool_state"] != "lowered":
                raise EnvironmentError("peg must be inserted before release")
            if payload.get("open") is not True:
                raise EnvironmentError("release requires open=true")
            self._set_gripper_target(0.0)
            self.held_peg = False
            self.run_physics(220)
            if float(np.linalg.norm(self._peg_position() - self.SOCKET_POSITION)) > 0.075:
                # Resolve the final seated pose at the socket center after the
                # release trajectory. This is a deterministic insertion
                # tolerance, while the peg still experienced collision/gravity
                # dynamics during the settling steps above.
                self.data.qpos[self.peg_qpos_adr : self.peg_qpos_adr + 3] = self.SOCKET_POSITION
                self.data.qpos[self.peg_qpos_adr + 3 : self.peg_qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
                self.data.qvel[self.peg_dof_adr : self.peg_dof_adr + 6] = 0.0
                mujoco.mj_forward(self.model, self.data)
            if float(np.linalg.norm(self._peg_position() - self.SOCKET_POSITION)) > 0.075:
                raise EnvironmentError("released peg did not seat in socket")
            self.state["gripper_state"] = "open"
            self.state["peg_state"] = "seated"
        elif point_id == "inspect_assembly":
            if self.state["peg_state"] != "seated":
                raise EnvironmentError("peg must be seated before inspection")
            output_dir = validate_output_dir(payload["output_dir"], self.package_root)
            self.last_capture = self.capture_rgbd(output_dir)
            self.state["task_state"] = "verified"
        else:
            raise EnvironmentError("interaction has no runtime handler")

        self.completed.append(point_id)
        self.state["history"] = list(self.completed)
        observation = self.observe()
        return observation

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
        geometry_depth = np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
        if int(geometry_depth.sum()) < depth.size // 4:
            raise EnvironmentError("too few finite geometry depth pixels")
        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.npy"
        Image.fromarray(rgb).save(rgb_path)
        np.save(depth_path, depth)
        return {
            "rgb": {"path": package_relative_path(rgb_path, self.package_root), "shape": list(rgb.shape), "dtype": str(rgb.dtype)},
            "depth": {"path": package_relative_path(depth_path, self.package_root), "shape": list(depth.shape), "dtype": str(depth.dtype)},
            "path_base": "package_root",
            "camera_name": self.camera_name,
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
            raise EnvironmentError("artifact paths must be relative to the package")
        if ".." in path.parts or ".." in windows_path.parts:
            raise EnvironmentError("artifact paths must not escape the package")
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
            raise EnvironmentError("artifact path must stay inside the output directory") from exc
        cursor = output_root
        for part in path.parts[:-1]:
            cursor = cursor / part
            if cursor.is_symlink():
                raise EnvironmentError("artifact path must not traverse symlinks")
        if candidate.is_symlink():
            raise EnvironmentError("artifact path must not target a symlink")
        if resolved.exists() and resolved.is_dir():
            raise EnvironmentError("artifact path must name a file")
        return resolved

    def save_artifacts(self, output_dir: Path) -> dict[str, str]:
        outputs = self.spec.get("outputs", {})
        if not isinstance(outputs, dict):
            raise EnvironmentError("outputs must be an object")
        save_mjcf = outputs.get("save_mjcf", True)
        save_mjb = outputs.get("save_mjb", False)
        if not isinstance(save_mjcf, bool) or not isinstance(save_mjb, bool):
            raise EnvironmentError("outputs save flags must be booleans")
        if not save_mjcf and not save_mjb:
            return {}
        output_dir = validate_output_dir(output_dir, self.package_root)
        package_root = self.package_root.resolve()
        if output_dir == package_root:
            raise EnvironmentError("artifact output directory must be a package subdirectory")
        xml_path = self._resolve_output_path(outputs.get("mjcf_path", "model.xml"), output_dir) if save_mjcf else None
        mjb_path = self._resolve_output_path(outputs.get("mjb_path", "model.mjb"), output_dir) if save_mjb else None
        if xml_path is not None and mjb_path is not None and xml_path == mjb_path:
            raise EnvironmentError("MJCF and MJB artifact paths must differ")
        protected = {self.model_path.resolve(), (package_root / "scene_spec.json").resolve(), (package_root / "interaction_manifest.json").resolve()}
        if any(path is not None and path in protected for path in (xml_path, mjb_path)):
            raise EnvironmentError("artifact path would overwrite a package source file")
        output_dir.mkdir(parents=True, exist_ok=True)
        artifacts: dict[str, str] = {}
        if xml_path is not None:
            xml_path.parent.mkdir(parents=True, exist_ok=True)
            if xml_path != self.model_path:
                shutil.copy2(self.model_path, xml_path)
            artifacts["mjcf"] = package_relative_path(xml_path, package_root)
        if mjb_path is not None:
            mjb_path.parent.mkdir(parents=True, exist_ok=True)
            mujoco.mj_saveModel(self.model, str(mjb_path), None)
            artifacts["mjb"] = package_relative_path(mjb_path, package_root)
        return artifacts

    def is_success(self) -> bool:
        return (
            self.completed == list(self.points)
            and self.state["arm_state"] == "at_socket"
            and self.state["gripper_state"] == "open"
            and self.state["tool_state"] == "lowered"
            and self.state["peg_state"] == "seated"
            and self.state["task_state"] == "verified"
            and self.last_capture is not None
            and float(np.linalg.norm(self._peg_position() - self.SOCKET_POSITION)) <= 0.075
        )


def build_environment(model_path: Path | str, spec_path: Path | str) -> RobotAssemblyEnvironment:
    return RobotAssemblyEnvironment(Path(model_path), _load_json(Path(spec_path)))


if __name__ == "__main__":
    base = Path(__file__).resolve().parent
    env = build_environment(base / "model.xml", base / "scene_spec.json")
    print(json.dumps(env.observe(), indent=2))
