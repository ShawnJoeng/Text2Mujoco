#!/usr/bin/env python3
"""Executable interaction contract for the warehouse navigation scene."""

from __future__ import annotations

import copy
import json
import math
import os
import shutil
from pathlib import Path, PureWindowsPath
from typing import Any, Mapping, Sequence

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


def xyzw_to_wxyz(value: Sequence[float]) -> list[float]:
    if len(value) != 4 or not all(math.isfinite(float(item)) for item in value):
        raise EnvironmentError("orientation_xyzw must contain four finite numbers")
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
    unknown = set(payload) - set(properties)
    if unknown:
        raise EnvironmentError(f"unknown payload fields: {sorted(unknown)}")
    missing = [name for name in required if name not in payload]
    if missing:
        raise EnvironmentError(f"missing payload fields: {missing}")
    for name, value in payload.items():
        rule = properties[name]
        expected = rule.get("type")
        if expected == "boolean":
            if not isinstance(value, bool):
                raise EnvironmentError(f"payload.{name} must be boolean")
        elif expected == "string":
            if not isinstance(value, str) or not value.strip():
                raise EnvironmentError(f"payload.{name} must be a non-empty string")
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
                raise EnvironmentError(f"payload.{name} must be a finite {expected}")
            if "minimum" in rule and value < rule["minimum"]:
                raise EnvironmentError(f"payload.{name} is below minimum")
            if "maximum" in rule and value > rule["maximum"]:
                raise EnvironmentError(f"payload.{name} is above maximum")
        elif expected == "array":
            if not isinstance(value, list):
                raise EnvironmentError(f"payload.{name} must be an array")
            minimum = rule.get("minItems", 0)
            maximum = rule.get("maxItems", 1 << 30)
            if not minimum <= len(value) <= maximum:
                raise EnvironmentError(f"payload.{name} length is out of bounds")
            item_schema = rule.get("items", {})
            if not isinstance(item_schema, dict):
                raise EnvironmentError(f"payload.{name} has a malformed item schema")
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
                        raise EnvironmentError(f"payload.{name} must contain finite {item_type}s")
    return dict(payload)


class WarehouseNavigationEnv:
    START = np.array([-2.35, -1.35], dtype=float)
    CHECKPOINT_A = np.array([-1.55, 0.75], dtype=float)
    CHECKPOINT_B = np.array([1.55, 0.80], dtype=float)
    SOUTH_BYPASS = (
        np.array([-1.25, -1.35], dtype=float),
        np.array([0.75, -1.35], dtype=float),
        np.array([1.15, -0.95], dtype=float),
        CHECKPOINT_B,
    )
    ROBOT_RADIUS = 0.23
    SHELF_RECTS = {
        "central_shelf_geom": (-0.35, 0.35, -0.85, 1.35),
        "north_shelf_geom": (-2.60, -0.90, 1.445, 1.995),
        "east_shelf_geom": (2.075, 2.625, -0.875, 0.375),
    }

    def __init__(self, model_path: Path, spec_path: Path):
        self.model_path = Path(model_path).resolve()
        self.spec_path = Path(spec_path).resolve()
        self.package_root = self.spec_path.parent
        self.spec = _load_json(self.spec_path)
        self.manifest = _load_json(self.spec_path.with_name("interaction_manifest.json"))
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self._points = {point["id"]: point for point in self.manifest["interaction_points"]}
        self._resolve_contract_names()
        self._validate_manifest_parity()
        camera_sensor = next(
            sensor for sensor in self.spec["sensors"] if sensor.get("type") == "camera"
        )
        self.camera_name = str(camera_sensor["camera_name"])
        self.camera_id = self._id("camera", self.camera_name)
        self.default_seed = int(self.spec["scene"]["world"]["seed"])
        self.seed = self.default_seed
        self.rng = np.random.default_rng(self.seed)
        self.completed: set[str] = set()
        self.history: list[str] = []
        self.route_trace: list[list[float]] = []
        self.shelf_collision_count = 0
        self.camera_inspection_complete = False
        self.last_sensor_observation: dict[str, Any] | None = None
        self.reset()

    def _id(self, object_type: str, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, OBJECT_TYPES[object_type], name)
        if object_id < 0:
            raise EnvironmentError(f"missing MuJoCo {object_type}: {name}")
        return int(object_id)

    def _resolve_contract_names(self) -> None:
        for point in self.manifest["interaction_points"]:
            self._id(point["target"]["type"], point["target"]["name"])
            self._id("site", point["marker_site"])
        self.robot_body_id = self._id("body", "mobile_robot")
        self.robot_geom_id = self._id("geom", "mobile_robot_geom")
        self.robot_x_joint_id = self._id("joint", "robot_x")
        self.robot_y_joint_id = self._id("joint", "robot_y")
        self.robot_x_qpos = int(self.model.jnt_qposadr[self.robot_x_joint_id])
        self.robot_y_qpos = int(self.model.jnt_qposadr[self.robot_y_joint_id])
        self.robot_x_actuator_id = self._id("actuator", "robot_x_motor")
        self.robot_y_actuator_id = self._id("actuator", "robot_y_motor")
        self.shelf_geom_ids = {self._id("geom", name) for name in self.SHELF_RECTS}

    def _validate_manifest_parity(self) -> None:
        if self.manifest.get("schema_version") != self.spec.get("schema_version"):
            raise EnvironmentError("manifest/spec schema versions differ")
        if self.manifest.get("backend") != self.spec.get("backend"):
            raise EnvironmentError("manifest/spec backends differ")
        spec_points = {item["id"]: item for item in self.spec["interaction_points"]}
        if set(spec_points) != set(self._points):
            raise EnvironmentError("manifest/spec interaction IDs differ")
        for point_id, manifest_point in self._points.items():
            spec_point = spec_points[point_id]
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
                "action",
            ):
                if manifest_point[field] != spec_point[field]:
                    raise EnvironmentError(f"manifest/spec mismatch for {point_id}.{field}")
        order = self.manifest.get("dependency_order")
        if order is not None and order != [point["id"] for point in self.spec["interaction_points"]]:
            raise EnvironmentError("manifest dependency_order does not match scene spec")

    def list_interaction_points(self) -> list[dict[str, Any]]:
        return copy.deepcopy(self.manifest["interaction_points"])

    def get_action_schema(self) -> dict[str, Any]:
        return {
            point_id: copy.deepcopy(point["action"]["schema"])
            for point_id, point in self._points.items()
        }

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int) or seed < 0):
            raise EnvironmentError("seed must be a non-negative integer or None")
        self.seed = self.default_seed if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[self.robot_x_actuator_id] = 0.0
        self.data.ctrl[self.robot_y_actuator_id] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.completed = set()
        self.history = []
        self.route_trace = [self.robot_xy.tolist()]
        self.shelf_collision_count = 0
        self.camera_inspection_complete = False
        self.last_sensor_observation = None
        return self.observe()

    @property
    def robot_xy(self) -> np.ndarray:
        return np.asarray(self.data.xpos[self.robot_body_id, :2], dtype=float).copy()

    def _shelf_contacts(self) -> list[tuple[int, int]]:
        result = []
        for contact in self.data.contact[: self.data.ncon]:
            pair = {int(contact.geom1), int(contact.geom2)}
            if self.robot_geom_id in pair and pair.intersection(self.shelf_geom_ids):
                result.append((int(contact.geom1), int(contact.geom2)))
        return result

    @staticmethod
    def _point_rect_distance(point: np.ndarray, rect: tuple[float, float, float, float]) -> float:
        xmin, xmax, ymin, ymax = rect
        dx = max(xmin - point[0], 0.0, point[0] - xmax)
        dy = max(ymin - point[1], 0.0, point[1] - ymax)
        return math.hypot(dx, dy)

    def route_clearance(self, points: Sequence[np.ndarray]) -> dict[str, Any]:
        samples: list[np.ndarray] = []
        for start, end in zip(points[:-1], points[1:]):
            distance = float(np.linalg.norm(end - start))
            count = max(2, int(math.ceil(distance / 0.02)) + 1)
            samples.extend(start + (end - start) * fraction for fraction in np.linspace(0.0, 1.0, count))
        minimum = math.inf
        closest_shelf = ""
        for sample in samples:
            if not (-2.72 <= sample[0] <= 2.72 and -1.97 <= sample[1] <= 1.97):
                raise EnvironmentError(f"route leaves collision-safe warehouse bounds at {sample.tolist()}")
            for name, rect in self.SHELF_RECTS.items():
                clearance = self._point_rect_distance(sample, rect) - self.ROBOT_RADIUS
                if clearance < minimum:
                    minimum = clearance
                    closest_shelf = name
        if minimum < 0.03:
            raise EnvironmentError(f"route is not collision-safe: {minimum:.4f} m at {closest_shelf}")
        return {"minimum_clearance_m": float(minimum), "closest_shelf": closest_shelf, "sample_count": len(samples)}

    def _drive_waypoints(self, waypoints: Sequence[np.ndarray], speed_mps: float) -> None:
        route = [self.robot_xy] + [np.asarray(point, dtype=float) for point in waypoints]
        self.route_clearance(route)
        for target in route[1:]:
            start = self.robot_xy
            distance = float(np.linalg.norm(target - start))
            increments = max(1, int(math.ceil(distance / 0.08)))
            for fraction in np.linspace(1.0 / increments, 1.0, increments):
                command = start + (target - start) * fraction
                self.data.ctrl[self.robot_x_actuator_id] = command[0] - self.START[0]
                self.data.ctrl[self.robot_y_actuator_id] = command[1] - self.START[1]
                steps = max(8, int(math.ceil(0.08 / (speed_mps * self.model.opt.timestep))))
                for _ in range(steps):
                    mujoco.mj_step(self.model, self.data)
                    contacts = self._shelf_contacts()
                    if contacts:
                        self.shelf_collision_count += len(contacts)
                        raise EnvironmentError(f"robot contacted shelf: {contacts}")
                    if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
                        raise EnvironmentError("non-finite MuJoCo state during navigation")
                self.route_trace.append(self.robot_xy.tolist())
            for _ in range(120):
                mujoco.mj_step(self.model, self.data)
                if self._shelf_contacts():
                    self.shelf_collision_count += 1
                    raise EnvironmentError("robot contacted shelf while settling")
            if float(np.linalg.norm(self.robot_xy - target)) > 0.10:
                raise EnvironmentError(f"controller did not reach waypoint {target.tolist()}: {self.robot_xy.tolist()}")

    def step(self, action: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(action, Mapping) or set(action) != {"id", "payload"}:
            raise EnvironmentError("action must contain exactly id and payload")
        point_id = action["id"]
        if not isinstance(point_id, str) or point_id not in self._points:
            raise EnvironmentError(f"unknown interaction id: {point_id!r}")
        if point_id in self.completed:
            raise EnvironmentError(f"interaction already completed: {point_id}")
        point = self._points[point_id]
        missing = [dependency for dependency in point["depends_on"] if dependency not in self.completed]
        if missing:
            raise EnvironmentError(f"unmet dependencies for {point_id}: {missing}")
        payload = _validate_payload(action["payload"], point["action"]["schema"])
        sensor = None
        if point_id == "reach_checkpoint_a":
            self._drive_waypoints([self.CHECKPOINT_A], float(payload.get("speed_mps", 0.7)))
        elif point_id == "reach_checkpoint_b":
            self._drive_waypoints(self.SOUTH_BYPASS, float(payload.get("speed_mps", 0.7)))
        elif point_id == "inspect_top_camera":
            sensor = self.capture_rgbd(
                validate_output_dir(payload["output_dir"], self.package_root), "after"
            )
            self.camera_inspection_complete = True
            self.last_sensor_observation = sensor
        self.completed.add(point_id)
        self.history.append(point_id)
        observation = self.observe()
        if sensor is not None:
            observation["sensor"] = sensor
        return observation

    def observe(self) -> dict[str, Any]:
        robot = self.robot_xy
        return {
            "time_s": float(self.data.time),
            "seed": self.seed,
            "robot_position": [float(robot[0]), float(robot[1]), float(self.data.xpos[self.robot_body_id, 2])],
            "distance_to_a_m": float(np.linalg.norm(robot - self.CHECKPOINT_A)),
            "distance_to_b_m": float(np.linalg.norm(robot - self.CHECKPOINT_B)),
            "state": {
                "robot_state": "at_checkpoint_b" if "reach_checkpoint_b" in self.completed else "at_checkpoint_a" if "reach_checkpoint_a" in self.completed else "at_start",
                "checkpoint_a_reached": "reach_checkpoint_a" in self.completed,
                "checkpoint_b_reached": "reach_checkpoint_b" in self.completed,
                "camera_inspection_complete": self.camera_inspection_complete,
                "history": list(self.history),
                "shelf_collision_count": self.shelf_collision_count,
            },
            "route_trace": copy.deepcopy(self.route_trace),
            "sensor": copy.deepcopy(self.last_sensor_observation),
        }

    def is_success(self) -> bool:
        return (
            self.completed == set(self._points)
            and self.camera_inspection_complete
            and self.last_sensor_observation is not None
            and self.shelf_collision_count == 0
            and float(np.linalg.norm(self.robot_xy - self.CHECKPOINT_B)) <= 0.10
        )

    def run_physics(self, steps: int) -> None:
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 0:
            raise EnvironmentError("steps must be a non-negative integer")
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            raise EnvironmentError("non-finite state after physics stepping")

    def capture_rgbd(self, output_dir: Path, stem: str) -> dict[str, Any]:
        backend = os.environ.get("MUJOCO_GL", "unset")
        if backend in {"unset", "disable"}:
            raise EnvironmentError("RGB-D capture requires an enabled MUJOCO_GL backend")
        if not isinstance(stem, str) or not stem or Path(stem).name != stem:
            raise EnvironmentError("capture stem must be a simple file name")
        from PIL import Image

        width, height = (int(value) for value in self.spec["outputs"]["resolution"])
        output_dir = validate_output_dir(output_dir, self.package_root)
        output_dir.mkdir(parents=True, exist_ok=True)
        rgb_path = output_dir / f"{stem}.png"
        depth_path = output_dir / f"{stem}_depth.npy"
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
        Image.fromarray(rgb).save(rgb_path)
        np.save(depth_path, depth)
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
        if path.is_absolute() or (os.name != "nt" and (windows_path.drive or requested.startswith("\\"))):
            raise EnvironmentError("artifact paths must be relative to the package")
        if ".." in path.parts or ".." in windows_path.parts:
            raise EnvironmentError("artifact paths must not escape the package with '..'")
        parts = path.parts
        if parts and parts[0] == "output":
            path = Path(*parts[1:])
        return output_dir / path

    def save_artifacts(self, output_dir: Path) -> dict[str, str]:
        outputs = self.spec.get("outputs", {})
        artifacts: dict[str, str] = {}
        save_mjcf = bool(outputs.get("save_mjcf", True))
        save_mjb = bool(outputs.get("save_mjb", False))
        if not save_mjcf and not save_mjb:
            return artifacts
        output_dir = validate_output_dir(output_dir, self.package_root)
        output_dir.mkdir(parents=True, exist_ok=True)
        if save_mjcf:
            xml_path = self._resolve_output_path(str(outputs.get("mjcf_path", "model.xml")), output_dir)
            if xml_path.resolve() in {
                self.model_path,
                self.package_root / "scene_spec.json",
                self.package_root / "interaction_manifest.json",
            }:
                raise EnvironmentError("artifact path would overwrite a package source file")
            xml_path.parent.mkdir(parents=True, exist_ok=True)
            if self.model_path != xml_path.resolve():
                shutil.copy2(self.model_path, xml_path)
            artifacts["mjcf"] = package_relative_path(xml_path, self.package_root)
        if save_mjb:
            mjb_path = self._resolve_output_path(str(outputs.get("mjb_path", "model.mjb")), output_dir)
            if mjb_path.resolve() in {
                self.model_path,
                self.package_root / "scene_spec.json",
                self.package_root / "interaction_manifest.json",
            }:
                raise EnvironmentError("artifact path would overwrite a package source file")
            mjb_path.parent.mkdir(parents=True, exist_ok=True)
            mujoco.mj_saveModel(self.model, str(mjb_path), None)
            artifacts["mjb"] = package_relative_path(mjb_path, self.package_root)
        return artifacts


def build_environment(model_path: Path | str, spec_path: Path | str) -> WarehouseNavigationEnv:
    return WarehouseNavigationEnv(Path(model_path), Path(spec_path))
