#!/usr/bin/env python3
"""Executable MuJoCo interaction contract for the lever-ball-ramp scene."""

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
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise EnvironmentError(f"{path} must contain a JSON object")
    return value


def package_relative_path(path: Path, package_root: Path) -> str:
    try:
        resolved = Path(path).resolve()
        root = package_root.resolve()
    except (OSError, RuntimeError, ValueError) as exc:
        raise EnvironmentError("artifact path is invalid") from exc
    try:
        return str(resolved.relative_to(root))
    except ValueError as exc:
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
        raise EnvironmentError("output_dir must be package-relative or an in-package POSIX path")
    candidate = raw if raw.is_absolute() else package_root / raw
    try:
        resolved = candidate.resolve()
    except (OSError, RuntimeError, ValueError) as exc:
        raise EnvironmentError("output_dir is invalid") from exc
    try:
        resolved.relative_to(package_root.resolve())
    except ValueError as exc:
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
        raise EnvironmentError(f"action payload has unknown fields: {unknown}")
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
            if expected == "integer":
                valid_number = isinstance(value, int) and not isinstance(value, bool)
            else:
                valid_number = isinstance(value, (int, float)) and not isinstance(value, bool)
            try:
                finite = math.isfinite(float(value))
            except (OverflowError, TypeError, ValueError):
                finite = False
            if not valid_number or not finite:
                raise EnvironmentError(f"payload.{field} must be a finite {expected}")
            if "minimum" in rule and value < rule["minimum"]:
                raise EnvironmentError(f"payload.{field} is below minimum")
            if "maximum" in rule and value > rule["maximum"]:
                raise EnvironmentError(f"payload.{field} is above maximum")
        elif expected == "array":
            if not isinstance(value, list):
                raise EnvironmentError(f"payload.{field} must be an array")
            minimum = rule.get("minItems", 0)
            maximum = rule.get("maxItems", 1 << 30)
            if not minimum <= len(value) <= maximum:
                raise EnvironmentError(f"payload.{field} length is out of bounds")
            item_schema = rule.get("items", {})
            if not isinstance(item_schema, dict):
                raise EnvironmentError(f"payload.{field} has a malformed item schema")
            item_type = item_schema.get("type")
            if item_type in {"number", "integer"}:
                for item in value:
                    valid_item = (
                        isinstance(item, int) and not isinstance(item, bool)
                        if item_type == "integer"
                        else isinstance(item, (int, float)) and not isinstance(item, bool)
                    )
                    try:
                        finite_item = math.isfinite(float(item))
                    except (OverflowError, TypeError, ValueError):
                        finite_item = False
                    if not valid_item or not finite_item:
                        raise EnvironmentError(f"payload.{field} must contain finite {item_type}s")
    return payload


class LeverBallRampEnvironment:
    def __init__(self, model_path: Path, spec: Mapping[str, Any]):
        self.model_path = Path(model_path).resolve()
        self.package_root = self.model_path.parent
        self.spec = copy.deepcopy(dict(spec))
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.points = {point["id"]: point for point in self.spec["interaction_points"]}
        self.manifest = _load_json(self.model_path.with_name("interaction_manifest.json"))
        self._validate_manifest_parity()
        self.assets = {asset["id"]: asset for asset in self.spec["assets"]}
        self.default_seed = int(self.spec["scene"]["world"]["seed"])

        lever = self.assets["blue_lever"]
        gate = self.assets["release_gate"]
        ball = self.assets["purple_ball"]
        tray = self.assets["target_tray"]
        camera = next(sensor for sensor in self.spec["sensors"] if sensor["type"] == "camera")
        self.lever_joint_id = self.require_id("joint", lever["physics"]["joint_name"])
        self.lever_actuator_id = self.require_id("actuator", lever["physics"]["actuator_name"])
        self.gate_joint_id = self.require_id("joint", gate["physics"]["joint_name"])
        self.gate_actuator_id = self.require_id("actuator", gate["physics"]["actuator_name"])
        self.ball_joint_id = self.require_id("joint", ball["physics"]["joint_name"])
        self.ball_body_id = self.require_id("body", ball["body_name"])
        self.ball_geom_id = self.require_id("geom", ball["geometry"]["geom_name"])
        self.tray_body_id = self.require_id("body", tray["body_name"])
        self.tray_bottom_geom_id = self.require_id("geom", tray["geometry"]["part_geom_names"][0])
        self.camera_name = camera["camera_name"]
        self.camera_id = self.require_id("camera", self.camera_name)
        for point in self.points.values():
            target = point["target"]
            self.require_id(target["type"], target["name"])
            self.require_id("site", point["marker_site"])

        self.lever_qpos_adr = int(self.model.jnt_qposadr[self.lever_joint_id])
        self.gate_qpos_adr = int(self.model.jnt_qposadr[self.gate_joint_id])
        self.ball_qpos_adr = int(self.model.jnt_qposadr[self.ball_joint_id])
        self.ball_dof_adr = int(self.model.jnt_dofadr[self.ball_joint_id])
        self.tray_asset = tray
        self.reset()

    def require_id(self, object_type: str, name: str) -> int:
        if object_type not in OBJECT_TYPES:
            raise EnvironmentError(f"unsupported MuJoCo object type: {object_type}")
        object_id = int(mujoco.mj_name2id(self.model, OBJECT_TYPES[object_type], name))
        if object_id < 0:
            raise EnvironmentError(f"missing MuJoCo {object_type}: {name}")
        return object_id

    def list_interaction_points(self) -> list[dict[str, Any]]:
        return [copy.deepcopy(point) for point in self.spec["interaction_points"]]

    def get_action_schema(self) -> dict[str, dict[str, Any]]:
        return {
            point_id: copy.deepcopy(point["action"]["schema"])
            for point_id, point in self.points.items()
        }

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        if seed is not None and (
            not isinstance(seed, int) or isinstance(seed, bool) or seed < 0
        ):
            raise EnvironmentError("reset seed must be a non-negative integer")
        self.seed = self.default_seed if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[self.lever_actuator_id] = 0.0
        self.data.ctrl[self.gate_actuator_id] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.completed: list[str] = []
        self.state = {
            "lever_state": "ready",
            "gate_state": "closed",
            "release_zone_state": "occupied",
            "target_state": "empty",
            "inspection_state": "pending",
        }
        self.last_capture: dict[str, Any] | None = None
        return self.observe()

    def _validate_manifest_parity(self) -> None:
        if self.manifest.get("schema_version") != self.spec.get("schema_version"):
            raise EnvironmentError("manifest/spec schema versions differ")
        if self.manifest.get("backend") != self.spec.get("backend"):
            raise EnvironmentError("manifest/spec backends differ")
        manifest_points = self.manifest.get("interaction_points")
        if not isinstance(manifest_points, list):
            raise EnvironmentError("interaction_manifest.json must define interaction_points")
        spec_points = {point["id"]: point for point in self.spec["interaction_points"]}
        manifest_by_id = {point.get("id"): point for point in manifest_points}
        if set(spec_points) != set(manifest_by_id):
            raise EnvironmentError("manifest/spec interaction IDs differ")
        for point_id, spec_point in spec_points.items():
            manifest_point = manifest_by_id[point_id]
            for field in (
                "target",
                "marker_site",
                "pose",
                "affordance",
                "preconditions",
                "success_conditions",
                "depends_on",
                "effects",
                "reset",
            ):
                if manifest_point.get(field) != spec_point.get(field):
                    raise EnvironmentError(f"manifest/spec mismatch for {point_id}.{field}")
            if manifest_point.get("action") != spec_point["action"]:
                raise EnvironmentError(f"manifest/spec mismatch for {point_id}.action")
        order = self.manifest.get("dependency_order")
        if order is not None and order != [point["id"] for point in self.spec["interaction_points"]]:
            raise EnvironmentError("manifest dependency_order does not match scene spec")

    def run_physics(self, steps: int) -> None:
        if not isinstance(steps, int) or isinstance(steps, bool) or steps < 0:
            raise EnvironmentError("steps must be a non-negative integer")
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise EnvironmentError("simulation state became non-finite")

    def _ball_position(self) -> np.ndarray:
        return np.asarray(self.data.xpos[self.ball_body_id], dtype=float).copy()

    def _ball_speed(self) -> float:
        return float(np.linalg.norm(self.data.qvel[self.ball_dof_adr : self.ball_dof_adr + 3]))

    def ball_inside_target(self) -> bool:
        world = self._ball_position()
        origin = np.asarray(self.data.xpos[self.tray_body_id], dtype=float)
        rotation = np.asarray(self.data.xmat[self.tray_body_id], dtype=float).reshape(3, 3)
        local = rotation.T @ (world - origin)
        bounds = self.tray_asset["geometry"]["interior_bounds"]
        return bool(np.all(local > np.asarray(bounds["min"])) and np.all(local < np.asarray(bounds["max"])))

    def ball_contacts_tray_bottom(self) -> bool:
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            pair = {int(contact.geom1), int(contact.geom2)}
            if pair == {self.ball_geom_id, self.tray_bottom_geom_id}:
                return True
        return False

    def _marker_positions(self) -> dict[str, list[float]]:
        result = {}
        for point in self.spec["interaction_points"]:
            site_id = self.require_id("site", point["marker_site"])
            result[point["id"]] = self.data.site_xpos[site_id].astype(float).tolist()
        return result

    def observe(self) -> dict[str, Any]:
        position = self._ball_position()
        return {
            "time_s": float(self.data.time),
            "seed": self.seed,
            "lever_angle_rad": float(self.data.qpos[self.lever_qpos_adr]),
            "gate_lift_m": float(self.data.qpos[self.gate_qpos_adr]),
            "ball_position": position.tolist(),
            "ball_speed_mps": self._ball_speed(),
            "ball_has_cleared_release_zone": bool(position[0] > -0.25),
            "ball_inside_target": self.ball_inside_target(),
            "ball_contacts_tray_bottom": self.ball_contacts_tray_bottom(),
            "marker_positions": self._marker_positions(),
            "state": copy.deepcopy(self.state),
            "history": list(self.completed),
            "sensor": copy.deepcopy(self.last_capture),
        }

    def _require_dependencies(self, point: Mapping[str, Any]) -> None:
        missing = [dep for dep in point["depends_on"] if dep not in self.completed]
        if missing:
            raise EnvironmentError(f"unmet interaction dependencies: {missing}")

    def step(self, action: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(action, Mapping):
            raise EnvironmentError("action must be an object")
        if set(action) != {"id", "payload"}:
            raise EnvironmentError("action must contain exactly id and payload")
        point_id = action["id"]
        if not isinstance(point_id, str) or point_id not in self.points:
            raise EnvironmentError(f"unknown interaction id: {point_id!r}")
        if point_id in self.completed:
            raise EnvironmentError(f"interaction already completed: {point_id}")
        point = self.points[point_id]
        self._require_dependencies(point)
        payload = _validate_payload(action["payload"], point["action"]["schema"])

        sensor: dict[str, Any] | None = None
        if point_id == "pull_blue_lever":
            if self.state["lever_state"] != "ready":
                raise EnvironmentError("lever is not ready")
            pull_angle = float(payload.get("pull_angle_rad", 0.95))
            self.data.ctrl[self.lever_actuator_id] = -pull_angle
            self.data.ctrl[self.gate_actuator_id] = 0.13
            for _ in range(900):
                self.run_physics(1)
                if self.data.qpos[self.lever_qpos_adr] <= -0.75 and self.data.qpos[self.gate_qpos_adr] >= 0.1:
                    break
            if self.data.qpos[self.lever_qpos_adr] > -0.75 or self.data.qpos[self.gate_qpos_adr] < 0.1:
                raise EnvironmentError("lever/gate actuators did not reach the open state")
            self.state["lever_state"] = "pulled"
            self.state["gate_state"] = "open"
        elif point_id == "check_release_zone":
            if self.state["gate_state"] != "open":
                raise EnvironmentError("gate must be open before checking the release zone")
            max_steps = int(payload.get("max_steps", 1200))
            for _ in range(max_steps):
                if self._ball_position()[0] > -0.25:
                    break
                self.run_physics(1)
            if self._ball_position()[0] <= -0.25:
                raise EnvironmentError("ball did not clear the release zone")
            self.state["release_zone_state"] = "cleared"
        elif point_id == "confirm_target_tray":
            if self.state["release_zone_state"] != "cleared":
                raise EnvironmentError("release zone has not been cleared")
            max_steps = int(payload.get("max_steps", 3000))
            for _ in range(max_steps):
                if self.ball_inside_target() and self._ball_speed() < 0.15:
                    break
                self.run_physics(1)
            if not self.ball_inside_target() or self._ball_speed() >= 0.15:
                raise EnvironmentError(
                    f"ball did not settle in target tray: position={self._ball_position().tolist()}, speed={self._ball_speed():.5f}"
                )
            self.state["target_state"] = "occupied"
        elif point_id == "inspect_with_camera":
            if self.state["target_state"] != "occupied":
                raise EnvironmentError("target tray is not occupied")
            output_dir = validate_output_dir(payload["output_dir"], self.package_root)
            sensor = self.capture_rgbd(output_dir)
            self.state["inspection_state"] = "captured"
            self.last_capture = sensor
        else:
            raise EnvironmentError(f"no handler for interaction: {point_id}")

        self.completed.append(point_id)
        observation = self.observe()
        if sensor is not None:
            observation["sensor"] = sensor
        return observation

    def capture_rgbd(self, output_dir: Path) -> dict[str, Any]:
        from PIL import Image

        backend = os.environ.get("MUJOCO_GL", "unset")
        if backend in {"unset", "disable"}:
            raise EnvironmentError("RGB-D capture requires an enabled MUJOCO_GL backend")
        resolution = self.spec["outputs"].get("resolution", [640, 480])
        width, height = int(resolution[0]), int(resolution[1])
        output_dir = validate_output_dir(output_dir, self.package_root)
        output_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(self.model, height=height, width=width)
        context = getattr(renderer, "_gl_context", None)
        renderer_context = (
            f"{type(context).__module__}.{type(context).__name__}"
            if context is not None
            else "unknown"
        )
        try:
            renderer.update_scene(self.data, camera=self.camera_name)
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(self.data, camera=self.camera_name)
            depth = renderer.render().copy()
        finally:
            renderer.close()
        if rgb.shape != (height, width, 3) or rgb.dtype != np.uint8:
            raise EnvironmentError(f"unexpected RGB frame: shape={rgb.shape}, dtype={rgb.dtype}")
        if float(rgb.std()) < 5.0 or int(rgb.max()) - int(rgb.min()) < 50:
            raise EnvironmentError("RGB frame is blank or nearly uniform")
        if depth.shape != (height, width):
            raise EnvironmentError(f"unexpected depth frame shape: {depth.shape}")
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
        if (
            path.is_absolute()
            or windows_path.drive
            or windows_path.root
            or (path.parts and URI_SCHEME_PATTERN.match(path.parts[0]))
        ):
            raise EnvironmentError("artifact paths must be relative to the package")
        if ".." in path.parts or ".." in windows_path.parts:
            raise EnvironmentError("artifact paths must not escape the package with '..'")
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
        raw_save_mjcf = outputs.get("save_mjcf", True)
        raw_save_mjb = outputs.get("save_mjb", False)
        if not isinstance(raw_save_mjcf, bool) or not isinstance(raw_save_mjb, bool):
            raise EnvironmentError("outputs save flags must be booleans")
        artifacts: dict[str, str] = {}
        save_mjcf = raw_save_mjcf
        save_mjb = raw_save_mjb
        if not save_mjcf and not save_mjb:
            return artifacts
        output_dir = validate_output_dir(output_dir, self.package_root)
        package_root = self.package_root.resolve()
        if output_dir == package_root:
            raise EnvironmentError("artifact output directory must be a package subdirectory")
        xml_path: Path | None = None
        mjb_path: Path | None = None
        if save_mjcf:
            xml_path = self._resolve_output_path(
                outputs.get("mjcf_path", "model.xml"), output_dir
            )
        if save_mjb:
            mjb_path = self._resolve_output_path(
                outputs.get("mjb_path", "model.mjb"), output_dir
            )
            if xml_path is not None and mjb_path == xml_path:
                raise EnvironmentError("MJCF and MJB artifact paths must differ")

        protected_paths = {
            self.model_path.resolve(),
            (package_root / "scene_spec.json").resolve(),
            (package_root / "interaction_manifest.json").resolve(),
        }
        for artifact_path in (xml_path, mjb_path):
            if artifact_path is not None and artifact_path in protected_paths:
                raise EnvironmentError("artifact path would overwrite a package source file")

        output_dir.mkdir(parents=True, exist_ok=True)
        if xml_path is not None:
            xml_path.parent.mkdir(parents=True, exist_ok=True)
            if self.model_path != xml_path:
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
            and self.state["lever_state"] == "pulled"
            and self.state["gate_state"] == "open"
            and self.state["release_zone_state"] == "cleared"
            and self.state["target_state"] == "occupied"
            and self.state["inspection_state"] == "captured"
            and self.last_capture is not None
            and self.ball_inside_target()
        )


def build_environment(model_path: Path | str, spec_path: Path | str) -> LeverBallRampEnvironment:
    return LeverBallRampEnvironment(Path(model_path), _load_json(Path(spec_path)))


if __name__ == "__main__":
    base = Path(__file__).resolve().parent
    environment = build_environment(base / "model.xml", base / "scene_spec.json")
    print(json.dumps(environment.observe(), indent=2))
