#!/usr/bin/env python3
"""Executable interaction contract for the smart desktop drawer scene."""

from __future__ import annotations

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
    for required in schema.get("required", []):
        if required not in payload:
            raise EnvironmentError(f"action payload is missing required field: {required}")
    unknown = sorted(set(payload) - set(properties))
    if unknown:
        raise EnvironmentError(f"action payload has unknown fields: {unknown}")
    for key, value in payload.items():
        definition = properties[key]
        expected = definition.get("type")
        if expected == "string" and not isinstance(value, str):
            raise EnvironmentError(f"payload.{key} must be a string")
        if expected == "string" and not value.strip():
            raise EnvironmentError(f"payload.{key} must not be empty")
        if expected == "number":
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise EnvironmentError(f"payload.{key} must be a finite number")
            if "minimum" in definition and float(value) < float(definition["minimum"]):
                raise EnvironmentError(f"payload.{key} must be >= {definition['minimum']}")
            if "maximum" in definition and float(value) > float(definition["maximum"]):
                raise EnvironmentError(f"payload.{key} must be <= {definition['maximum']}")
    return dict(payload)


class SmartDrawerEnvironment:
    def __init__(self, model_path: Path, spec: Mapping[str, Any]):
        self.model_path = Path(model_path).resolve()
        self.spec = copy.deepcopy(dict(spec))
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.points = {point["id"]: point for point in self.spec["interaction_points"]}
        self.default_seed = int(self.spec["scene"]["world"]["seed"])
        self.seed = self.default_seed
        self.rng = np.random.default_rng(self.seed)

        self.unlock_joint_id = self.require_id("joint", "unlock_button_slide")
        self.unlock_actuator_id = self.require_id("actuator", "unlock_button_press")
        self.drawer_joint_id = self.require_id("joint", "drawer_slide")
        self.drawer_actuator_id = self.require_id("actuator", "drawer_pull")
        self.drawer_body_id = self.require_id("body", "tool_drawer")
        self.camera_id = self.require_id("camera", "fixed_inspection_camera")
        self.unlock_qpos_adr = int(self.model.jnt_qposadr[self.unlock_joint_id])
        self.drawer_qpos_adr = int(self.model.jnt_qposadr[self.drawer_joint_id])
        self.drawer_dof_adr = int(self.model.jnt_dofadr[self.drawer_joint_id])

        for point in self.points.values():
            target = point["target"]
            self.require_id(target["type"], target["name"])
            self.require_id("site", point["marker_site"])
        self.reset()

    def require_id(self, object_type: str, name: str) -> int:
        if object_type not in OBJECT_TYPES:
            raise EnvironmentError(f"unsupported object type: {object_type}")
        object_id = int(mujoco.mj_name2id(self.model, OBJECT_TYPES[object_type], name))
        if object_id < 0:
            raise EnvironmentError(f"missing MuJoCo {object_type}: {name}")
        return object_id

    def list_interaction_points(self) -> list[dict[str, Any]]:
        return [copy.deepcopy(point) for point in self.spec["interaction_points"]]

    def get_action_schema(self) -> dict[str, Any]:
        return {
            point_id: copy.deepcopy(point["action"]["schema"])
            for point_id, point in self.points.items()
        }

    def reset(self, seed: int | None = None) -> None:
        if seed is not None and (
            isinstance(seed, bool) or not isinstance(seed, int) or seed < 0
        ):
            raise EnvironmentError("reset seed must be a non-negative integer")
        self.seed = self.default_seed if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.ctrl[self.unlock_actuator_id] = 0.0
        self.data.ctrl[self.drawer_actuator_id] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.state = {
            "lock_state": "locked",
            "drawer_state": "closed",
            "inspection_state": "pending",
            "history": [],
        }

    def _run_until(self, predicate, *, max_steps: int, label: str) -> None:
        for _ in range(max_steps):
            mujoco.mj_step(self.model, self.data)
            if predicate():
                return
        raise EnvironmentError(f"{label} did not converge")

    def run_physics(self, steps: int) -> None:
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 0:
            raise EnvironmentError("steps must be a non-negative integer")
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise EnvironmentError("non-finite MuJoCo state")

    def step(self, action: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(action, dict):
            raise EnvironmentError("action must be an object")
        point_id = action.get("id")
        if not isinstance(point_id, str) or point_id not in self.points:
            raise EnvironmentError(f"unknown interaction id: {point_id!r}")
        point = self.points[point_id]
        payload = _validate_payload(action.get("payload", {}), point["action"]["schema"])
        missing = [dependency for dependency in point["depends_on"] if dependency not in self.state["history"]]
        if missing:
            raise EnvironmentError(f"unmet interaction dependencies: {missing}")

        if point_id == "press_unlock_button":
            if self.state["lock_state"] != "locked":
                raise EnvironmentError("drawer is already unlocked")
            depth = float(payload.get("press_depth_m", 0.012))
            self.data.ctrl[self.unlock_actuator_id] = depth
            self._run_until(
                lambda: float(self.data.qpos[self.unlock_qpos_adr]) >= depth - 0.001,
                max_steps=500,
                label="unlock button",
            )
            if float(self.data.qpos[self.unlock_qpos_adr]) < 0.01:
                raise EnvironmentError("button did not reach unlock depth")
            self.state["lock_state"] = "unlocked"

        elif point_id == "pull_drawer_22cm":
            if self.state["lock_state"] != "unlocked":
                raise EnvironmentError("drawer must be unlocked before pulling")
            if self.state["drawer_state"] != "closed":
                raise EnvironmentError("drawer is not closed")
            distance = float(payload["distance_m"])
            self.data.ctrl[self.drawer_actuator_id] = distance
            self._run_until(
                lambda: abs(float(self.data.qpos[self.drawer_qpos_adr]) - distance) <= 0.002,
                max_steps=2000,
                label="drawer pull",
            )
            self.state["drawer_state"] = "open_22cm"

        elif point_id == "inspect_open_drawer":
            if self.state["drawer_state"] != "open_22cm":
                raise EnvironmentError("drawer must be open 22 cm before inspection")
            if float(self.data.qpos[self.drawer_qpos_adr]) < 0.218:
                raise EnvironmentError("drawer position is below the inspection threshold")
            capture = self.capture_rgbd(Path(payload["output_dir"]))
            self.state["inspection_state"] = "verified_open"
            self.state["history"].append(point_id)
            observation = self.observe()
            observation["sensor"] = capture
            return observation

        self.state["history"].append(point_id)
        return self.observe()

    def observe(self) -> dict[str, Any]:
        return {
            "time_s": float(self.data.time),
            "seed": self.seed,
            "unlock_button_joint_qpos": float(self.data.qpos[self.unlock_qpos_adr]),
            "drawer_joint_qpos": float(self.data.qpos[self.drawer_qpos_adr]),
            "drawer_joint_velocity_mps": float(self.data.qvel[self.drawer_dof_adr]),
            "drawer_body_position": self.data.xpos[self.drawer_body_id].astype(float).tolist(),
            "state": copy.deepcopy(self.state),
        }

    def is_success(self) -> bool:
        return (
            self.state["lock_state"] == "unlocked"
            and self.state["drawer_state"] == "open_22cm"
            and self.state["inspection_state"] == "verified_open"
            and abs(float(self.data.qpos[self.drawer_qpos_adr]) - 0.22) <= 0.002
        )

    def capture_rgbd(self, output_dir: Path) -> dict[str, Any]:
        width, height = (int(value) for value in self.spec["outputs"]["resolution"])
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(self.model, height=height, width=width)
        try:
            renderer.update_scene(self.data, camera="fixed_inspection_camera")
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(self.data, camera="fixed_inspection_camera")
            depth = renderer.render().copy()
        finally:
            renderer.close()
        from PIL import Image

        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.npy"
        Image.fromarray(rgb).save(rgb_path)
        np.save(depth_path, depth)
        return {
            "rgb": {"path": str(rgb_path.resolve()), "shape": list(rgb.shape), "dtype": str(rgb.dtype)},
            "depth": {"path": str(depth_path.resolve()), "shape": list(depth.shape), "dtype": str(depth.dtype)},
            "camera": "fixed_inspection_camera",
            "renderer_context": os.environ.get("MUJOCO_GL", "unset"),
            "drawer_joint_qpos": float(self.data.qpos[self.drawer_qpos_adr]),
        }

    def save_artifacts(self, output_dir: Path) -> dict[str, str]:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        artifacts: dict[str, str] = {}
        if self.spec["outputs"].get("save_mjcf"):
            xml_path = output_dir / "model.xml"
            shutil.copy2(self.model_path, xml_path)
            artifacts["mjcf"] = str(xml_path.resolve())
        if self.spec["outputs"].get("save_mjb"):
            mjb_path = output_dir / "model.mjb"
            mujoco.mj_saveModel(self.model, str(mjb_path), None)
            artifacts["mjb"] = str(mjb_path.resolve())
        return artifacts


def build_environment(model_path: Path | str, spec_path: Path | str) -> SmartDrawerEnvironment:
    return SmartDrawerEnvironment(Path(model_path), _load_json(Path(spec_path)))


if __name__ == "__main__":
    base = Path(__file__).resolve().parent
    env = build_environment(base / "model.xml", base / "scene_spec.json")
    print(json.dumps(env.observe(), indent=2, ensure_ascii=False))
