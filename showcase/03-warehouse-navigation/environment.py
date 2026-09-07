#!/usr/bin/env python3
"""Executable interaction contract for the warehouse navigation scene."""

from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
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
    unknown = set(payload) - set(properties)
    if unknown:
        raise EnvironmentError(f"unknown payload fields: {sorted(unknown)}")
    missing = [name for name in required if name not in payload]
    if missing:
        raise EnvironmentError(f"missing payload fields: {missing}")
    for name, value in payload.items():
        rule = properties[name]
        expected = rule.get("type")
        if expected == "number":
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise EnvironmentError(f"payload.{name} must be a finite number")
            if "minimum" in rule and value < rule["minimum"]:
                raise EnvironmentError(f"payload.{name} is below minimum")
            if "maximum" in rule and value > rule["maximum"]:
                raise EnvironmentError(f"payload.{name} is above maximum")
        elif expected == "string" and (not isinstance(value, str) or not value.strip()):
            raise EnvironmentError(f"payload.{name} must be a non-empty string")
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
        self.spec = _load_json(self.spec_path)
        self.manifest = _load_json(self.spec_path.with_name("interaction_manifest.json"))
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self._points = {point["id"]: point for point in self.manifest["interaction_points"]}
        self._resolve_contract_names()
        self._validate_manifest_parity()
        self.default_seed = int(self.spec["scene"]["world"]["seed"])
        self.seed = self.default_seed
        self.rng = np.random.default_rng(self.seed)
        self.completed: set[str] = set()
        self.history: list[str] = []
        self.route_trace: list[list[float]] = []
        self.shelf_collision_count = 0
        self.camera_inspection_complete = False
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
        spec_points = {item["id"]: item for item in self.spec["interaction_points"]}
        if set(spec_points) != set(self._points):
            raise EnvironmentError("manifest/spec interaction IDs differ")
        for point_id, manifest_point in self._points.items():
            spec_point = spec_points[point_id]
            for field in ("target", "marker_site", "depends_on", "action"):
                if manifest_point[field] != spec_point[field]:
                    raise EnvironmentError(f"manifest/spec mismatch for {point_id}.{field}")

    def list_interaction_points(self) -> list[dict[str, Any]]:
        return copy.deepcopy(self.manifest["interaction_points"])

    def get_action_schema(self) -> dict[str, Any]:
        return copy.deepcopy(self.manifest["action_envelope"])

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
            sensor = self.capture_rgbd(Path(payload["output_dir"]), "after")
            self.camera_inspection_complete = True
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
        }

    def is_success(self) -> bool:
        return (
            self.completed == set(self._points)
            and self.camera_inspection_complete
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
        from PIL import Image

        width, height = (int(value) for value in self.spec["outputs"]["resolution"])
        output_dir.mkdir(parents=True, exist_ok=True)
        rgb_path = output_dir / f"{stem}.png"
        depth_path = output_dir / f"{stem}_depth.npy"
        renderer = mujoco.Renderer(self.model, height=height, width=width)
        try:
            renderer.update_scene(self.data, camera="top_camera")
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(self.data, camera="top_camera")
            depth = renderer.render().copy()
        finally:
            renderer.close()
        Image.fromarray(rgb).save(rgb_path)
        np.save(depth_path, depth)
        return {
            "rgb": {"path": str(rgb_path.resolve()), "shape": list(rgb.shape), "dtype": str(rgb.dtype)},
            "depth": {"path": str(depth_path.resolve()), "shape": list(depth.shape), "dtype": str(depth.dtype)},
            "camera": "top_camera",
            "renderer_context": f"MuJoCo Renderer with MUJOCO_GL={backend}",
        }


def build_environment(model_path: Path | str, spec_path: Path | str) -> WarehouseNavigationEnv:
    return WarehouseNavigationEnv(Path(model_path), Path(spec_path))

