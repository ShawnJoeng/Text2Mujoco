#!/usr/bin/env python3
"""MuJoCo implementation of the generated interaction contract."""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import shutil
from pathlib import Path
from typing import Any, Mapping

import mujoco
import numpy as np


class EnvironmentError(RuntimeError):
    """Raised when the model or an action violates the generated contract."""


OBJECT_TYPES = {
    "body": mujoco.mjtObj.mjOBJ_BODY,
    "geom": mujoco.mjtObj.mjOBJ_GEOM,
    "joint": mujoco.mjtObj.mjOBJ_JOINT,
    "actuator": mujoco.mjtObj.mjOBJ_ACTUATOR,
    "site": mujoco.mjtObj.mjOBJ_SITE,
    "camera": mujoco.mjtObj.mjOBJ_CAMERA,
}


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise EnvironmentError(f"{path} must contain a JSON object")
    return value


def xyzw_to_wxyz(value: list[float]) -> list[float]:
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
    for field in required:
        if field not in payload:
            raise EnvironmentError(f"action payload is missing required field: {field}")
    unknown = sorted(set(payload) - set(properties))
    if unknown:
        raise EnvironmentError(f"action payload has unknown fields: {unknown}")
    for field, value in payload.items():
        field_schema = properties[field]
        expected = field_schema.get("type")
        if expected == "boolean" and not isinstance(value, bool):
            raise EnvironmentError(f"payload.{field} must be boolean")
        if expected == "string" and not isinstance(value, str):
            raise EnvironmentError(f"payload.{field} must be string")
        if expected == "number" and (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(float(value))
        ):
            raise EnvironmentError(f"payload.{field} must be a finite number")
        if expected == "number" and "minimum" in field_schema:
            if float(value) < float(field_schema["minimum"]):
                raise EnvironmentError(
                    f"payload.{field} must be >= {field_schema['minimum']}"
                )
        if expected == "number" and "maximum" in field_schema:
            if float(value) > float(field_schema["maximum"]):
                raise EnvironmentError(
                    f"payload.{field} must be <= {field_schema['maximum']}"
                )
        if expected == "array":
            if not isinstance(value, list):
                raise EnvironmentError(f"payload.{field} must be an array")
            minimum = int(field_schema.get("minItems", 0))
            maximum = int(field_schema.get("maxItems", 1 << 30))
            if not minimum <= len(value) <= maximum:
                raise EnvironmentError(
                    f"payload.{field} length must be between {minimum} and {maximum}"
                )
            if field_schema.get("items", {}).get("type") == "number" and not all(
                isinstance(item, (int, float))
                and not isinstance(item, bool)
                and math.isfinite(float(item))
                for item in value
            ):
                raise EnvironmentError(f"payload.{field} must contain finite numbers")
    return payload


class MujocoEnvironment:
    def __init__(self, model_path: Path, spec: Mapping[str, Any]):
        self.model_path = Path(model_path).resolve()
        self.spec = copy.deepcopy(dict(spec))
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.points = {point["id"]: point for point in self.spec["interaction_points"]}
        self.default_seed = int(self.spec["scene"]["world"]["seed"])
        self.seed = self.default_seed
        self.rng = np.random.default_rng(self.seed)

        self._assets = {asset["id"]: asset for asset in self.spec["assets"]}
        button_asset = self._assets["start_button"]
        cube_asset = self._assets["red_cube"]
        box_asset = self._assets["target_box"]
        camera_sensor = next(
            sensor for sensor in self.spec["sensors"] if sensor.get("type") == "camera"
        )
        self.camera_name = camera_sensor["camera_name"]
        self.button_joint_id = self.require_id(
            "joint", button_asset["physics"]["joint_name"]
        )
        self.button_actuator_id = self.require_id(
            "actuator", button_asset["physics"]["actuator_name"]
        )
        self.cube_joint_id = self.require_id("joint", cube_asset["physics"]["joint_name"])
        self.cube_body_id = self.require_id("body", cube_asset["body_name"])
        self.cube_geom_id = self.require_id("geom", cube_asset["geometry"]["geom_name"])
        self.box_body_id = self.require_id("body", box_asset["body_name"])
        self.box_bottom_geom_id = self.require_id(
            "geom", box_asset["geometry"]["part_geom_names"][0]
        )
        self.camera_id = self.require_id("camera", camera_sensor["camera_name"])
        for point in self.points.values():
            if point.get("marker_site"):
                self.require_id("site", point["marker_site"])
            target = point["target"]
            self.require_id(target["type"], target["name"])

        self.button_qpos_adr = int(self.model.jnt_qposadr[self.button_joint_id])
        self.cube_qpos_adr = int(self.model.jnt_qposadr[self.cube_joint_id])
        self.cube_dof_adr = int(self.model.jnt_dofadr[self.cube_joint_id])
        self.box_asset = box_asset
        self.reset()

    def require_id(self, object_type: str, name: str) -> int:
        if object_type not in OBJECT_TYPES:
            raise EnvironmentError(f"unsupported MuJoCo object type: {object_type}")
        value = int(mujoco.mj_name2id(self.model, OBJECT_TYPES[object_type], name))
        if value < 0:
            raise EnvironmentError(f"missing MuJoCo {object_type}: {name}")
        return value

    def list_interaction_points(self) -> list[dict[str, Any]]:
        return [copy.deepcopy(point) for point in self.spec["interaction_points"]]

    def get_action_schema(self) -> dict[str, Any]:
        return {
            point_id: copy.deepcopy(point["action"]["schema"])
            for point_id, point in self.points.items()
        }

    def reset(self, seed: int | None = None) -> None:
        if seed is not None and (
            not isinstance(seed, int)
            or isinstance(seed, bool)
            or seed < 0
        ):
            raise EnvironmentError("reset seed must be a non-negative integer")
        self.seed = self.default_seed if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.completed: set[str] = set()
        self.state: dict[str, Any] = {
            "button_state": "ready",
            "cube_state": "free",
            "cube_released": False,
            "last_observation_updated": False,
            "history": [],
        }
        self.last_sensor_observation: dict[str, Any] | None = None
        self.held_cube_qpos: np.ndarray | None = None

    def run_physics(self, steps: int) -> None:
        if not isinstance(steps, int) or isinstance(steps, bool) or steps < 0:
            raise EnvironmentError("steps must be a non-negative integer")
        for _ in range(steps):
            if self.held_cube_qpos is not None:
                self.data.qpos[self.cube_qpos_adr : self.cube_qpos_adr + 7] = (
                    self.held_cube_qpos
                )
                self.data.qvel[self.cube_dof_adr : self.cube_dof_adr + 6] = 0.0
            mujoco.mj_step(self.model, self.data)
        if self.held_cube_qpos is not None:
            self.data.qpos[self.cube_qpos_adr : self.cube_qpos_adr + 7] = (
                self.held_cube_qpos
            )
            self.data.qvel[self.cube_dof_adr : self.cube_dof_adr + 6] = 0.0
            mujoco.mj_forward(self.model, self.data)
        self._require_finite_state()

    def _require_finite_state(self) -> None:
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise EnvironmentError("MuJoCo state contains non-finite values")

    def _set_cube_pose(self, position: list[float]) -> None:
        if len(position) != 3 or not all(math.isfinite(float(value)) for value in position):
            raise EnvironmentError("cube position must contain three finite numbers")
        qpos = self.cube_qpos_adr
        dof = self.cube_dof_adr
        self.data.qpos[qpos : qpos + 3] = np.asarray(position, dtype=float)
        self.data.qpos[qpos + 3 : qpos + 7] = np.asarray(
            xyzw_to_wxyz([0.0, 0.0, 0.0, 1.0]), dtype=float
        )
        self.data.qvel[dof : dof + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _box_world_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        center = np.asarray(self.box_asset["pose"]["position"], dtype=float)
        bounds = self.box_asset["geometry"]["interior_bounds"]
        return center + np.asarray(bounds["min"]), center + np.asarray(bounds["max"])

    def cube_is_inside_box(self) -> bool:
        minimum, maximum = self._box_world_bounds()
        position = np.asarray(self.data.xpos[self.cube_body_id], dtype=float)
        return bool(np.all(position >= minimum) and np.all(position <= maximum))

    def cube_linear_speed(self) -> float:
        velocity = self.data.qvel[self.cube_dof_adr : self.cube_dof_adr + 3]
        return float(np.linalg.norm(velocity))

    def cube_contacts_box_bottom(self) -> bool:
        expected = frozenset((self.cube_geom_id, self.box_bottom_geom_id))
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if frozenset((int(contact.geom1), int(contact.geom2))) == expected:
                return True
        return False

    def _require_dependencies(self, point: Mapping[str, Any]) -> None:
        missing = [name for name in point["depends_on"] if name not in self.completed]
        if missing:
            raise EnvironmentError(
                f"interaction {point['id']} has unmet dependencies: {missing}"
            )

    def _require_state_preconditions(self, point_id: str) -> None:
        if point_id == "press_start_button" and self.state["button_state"] != "ready":
            raise EnvironmentError("press_start_button requires button_state='ready'")
        if point_id == "grasp_red_cube" and (
            self.state["button_state"] != "pressed" or self.state["cube_state"] != "free"
        ):
            raise EnvironmentError(
                "grasp_red_cube requires a pressed button and a free cube"
            )
        if point_id == "place_cube_in_box" and (
            self.state["button_state"] != "pressed"
            or self.state["cube_state"] != "grasped"
        ):
            raise EnvironmentError(
                "place_cube_in_box requires a pressed button and a grasped cube"
            )
        if point_id == "inspect_rgbd" and self.state["cube_state"] != "inside_box":
            raise EnvironmentError("inspect_rgbd requires cube_state='inside_box'")

    def step(self, action: Mapping[str, Any]) -> dict[str, Any]:
        if not isinstance(action, Mapping):
            raise EnvironmentError("action must be an object")
        point_id = action.get("id")
        if not isinstance(point_id, str) or point_id not in self.points:
            raise EnvironmentError(f"unknown interaction id: {point_id}")
        point = self.points[str(point_id)]
        self._require_dependencies(point)
        self._require_state_preconditions(str(point_id))
        payload = _validate_payload(action.get("payload", {}), point["action"]["schema"])

        if point_id == "press_start_button":
            press_depth = float(payload.get("press_depth_m", 0.018))
            target = -press_depth
            self.data.ctrl[self.button_actuator_id] = target
            self.run_physics(500)
            if float(self.data.qpos[self.button_qpos_adr]) > -0.014:
                raise EnvironmentError("button actuator did not reach the pressed range")
            self.state["button_state"] = "pressed"
        elif point_id == "grasp_red_cube":
            if payload.get("close") is not True:
                raise EnvironmentError("grasp_red_cube requires close=true")
            grasp_position = self.points["grasp_red_cube"]["pose"]["position"]
            self._set_cube_pose([float(value) for value in grasp_position])
            self.held_cube_qpos = self.data.qpos[
                self.cube_qpos_adr : self.cube_qpos_adr + 7
            ].copy()
            self.state["cube_state"] = "grasped"
            self.state["cube_released"] = False
        elif point_id == "place_cube_in_box":
            if payload.get("release") is not True:
                raise EnvironmentError("place_cube_in_box requires release=true")
            desired = np.asarray(payload["pose"], dtype=float)
            minimum, maximum = self._box_world_bounds()
            if not np.all(desired >= minimum) or not np.all(desired <= maximum):
                raise EnvironmentError(
                    f"placement pose {desired.tolist()} is outside target bounds"
                )
            self.held_cube_qpos = None
            # The requested z is the target center; release from above it so gravity
            # and the open-box colliders still determine the final resting pose.
            release_height = float(desired[2]) + 0.25
            self._set_cube_pose([float(desired[0]), float(desired[1]), release_height])
            self.run_physics(900)
            if not self.cube_is_inside_box():
                raise EnvironmentError(
                    f"released cube did not settle inside box: {self.observe()['cube_position']}"
                )
            if self.cube_linear_speed() >= 0.1:
                raise EnvironmentError("released cube did not settle below 0.1 m/s")
            self.state["cube_state"] = "inside_box"
            self.state["cube_released"] = True
        elif point_id == "inspect_rgbd":
            output_dir = Path(payload["output_dir"])
            self.last_sensor_observation = self.capture_rgbd(output_dir)
            self.state["last_observation_updated"] = True
        else:
            raise EnvironmentError(f"interaction has no runtime handler: {point_id}")

        self.completed.add(str(point_id))
        self.state["history"].append(str(point_id))
        return self.observe()

    def observe(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "time_s": float(self.data.time),
            "button_joint_qpos": float(self.data.qpos[self.button_qpos_adr]),
            "cube_position": [float(value) for value in self.data.xpos[self.cube_body_id]],
            "cube_linear_speed_mps": self.cube_linear_speed(),
            "cube_contacts_box_bottom": self.cube_contacts_box_bottom(),
            "state": copy.deepcopy(self.state),
            "sensor": copy.deepcopy(self.last_sensor_observation),
            "mujoco_gl": os.environ.get("MUJOCO_GL", "unset"),
        }

    def is_success(self) -> bool:
        return bool(
            self.state["button_state"] == "pressed"
            and self.state["cube_state"] == "inside_box"
            and self.state["cube_released"]
            and self.state["last_observation_updated"]
            and self.cube_is_inside_box()
        )

    def capture_rgbd(self, output_dir: Path) -> dict[str, Any]:
        from PIL import Image

        resolution = self.spec["outputs"].get("resolution", [640, 480])
        width, height = int(resolution[0]), int(resolution[1])
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
        far_m = float(self.model.vis.map.zfar * self.model.stat.extent)
        geometry_depth = (
            np.isfinite(depth) & (depth > 0) & (depth < far_m * 0.999)
        )
        if depth.shape != (height, width) or not geometry_depth.any():
            raise EnvironmentError(
                f"depth frame has no finite geometry values below far plane: {depth.shape}"
            )

        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.npy"
        preview_path = output_dir / "depth_preview.png"
        Image.fromarray(rgb, "RGB").save(rgb_path)
        np.save(depth_path, depth)

        lower, upper = np.percentile(depth[geometry_depth], [2, 98])
        span = max(float(upper - lower), 1e-6)
        normalized = np.clip((depth - lower) / span, 0.0, 1.0)
        preview = np.where(
            geometry_depth, (1.0 - normalized) * 255.0, 0.0
        ).astype(np.uint8)
        Image.fromarray(preview, "L").save(preview_path)

        return {
            "backend": os.environ.get("MUJOCO_GL", "unset"),
            "renderer_context": renderer_context,
            "camera_pose": {
                "position": [
                    float(value) for value in self.model.cam_pos[self.camera_id]
                ],
                "orientation_xyzw": [
                    float(self.model.cam_quat[self.camera_id][1]),
                    float(self.model.cam_quat[self.camera_id][2]),
                    float(self.model.cam_quat[self.camera_id][3]),
                    float(self.model.cam_quat[self.camera_id][0]),
                ],
            },
            "rgb": {
                "path": str(rgb_path.resolve()),
                "shape": list(rgb.shape),
                "mean": float(rgb.mean()),
                "std": float(rgb.std()),
            },
            "depth": {
                "path": str(depth_path.resolve()),
                "preview_path": str(preview_path.resolve()),
                "shape": list(depth.shape),
                "finite_geometry_pixels": int(geometry_depth.sum()),
                "min_m": float(depth[geometry_depth].min()),
                "max_m": float(depth[geometry_depth].max()),
                "far_plane_m": far_m,
            },
        }

    @staticmethod
    def _resolve_output_path(requested: str, output_dir: Path) -> Path:
        path = Path(requested)
        if path.is_absolute():
            return path
        # Specs commonly use ./output/foo; output_dir is the caller's output root.
        parts = path.parts
        if parts and parts[0] == "output":
            path = Path(*parts[1:])
        return output_dir / path

    def save_artifacts(self, output_dir: Path) -> dict[str, str]:
        outputs = self.spec.get("outputs", {})
        save_mjcf = bool(outputs.get("save_mjcf", True))
        save_mjb = bool(outputs.get("save_mjb", False))
        output_dir.mkdir(parents=True, exist_ok=True)
        artifacts: dict[str, str] = {}
        if save_mjcf:
            xml_path = self._resolve_output_path(
                str(outputs.get("mjcf_path", "model.xml")), output_dir
            )
            xml_path.parent.mkdir(parents=True, exist_ok=True)
            if self.model_path != xml_path.resolve():
                shutil.copy2(self.model_path, xml_path)
            artifacts["mjcf"] = str(xml_path.resolve())
        if save_mjb:
            mjb_path = self._resolve_output_path(
                str(outputs.get("mjb_path", "model.mjb")), output_dir
            )
            mjb_path.parent.mkdir(parents=True, exist_ok=True)
            mujoco.mj_saveModel(self.model, str(mjb_path), None)
            artifacts["mjb"] = str(mjb_path.resolve())
        return artifacts


def build_environment(model_path: Path, spec_path: Path) -> MujocoEnvironment:
    return MujocoEnvironment(Path(model_path), load_json(Path(spec_path)))


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--output-dir", type=Path, default=base / "output")
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()

    environment = build_environment(args.model, args.spec)
    environment.run_physics(args.steps)
    artifacts = environment.save_artifacts(args.output_dir)
    result: dict[str, Any] = {
        "mujoco_version": mujoco.__version__,
        "artifacts": artifacts,
        "observation": environment.observe(),
    }
    if args.render:
        result["rgbd"] = environment.capture_rgbd(args.output_dir / "screenshots" / "initial")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
