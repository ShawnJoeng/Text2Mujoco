#!/usr/bin/env python3
"""Physics and interaction smoke test for conveyor-to-arm handoff."""

from __future__ import annotations

import argparse
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Callable

import mujoco
import numpy as np

from environment import ConveyorArmEnvironment, EnvironmentError, build_environment, xyzw_to_wxyz


def expect_error(callback: Callable[[], Any], label: str) -> None:
    try:
        callback()
    except EnvironmentError:
        return
    raise AssertionError(f"expected EnvironmentError: {label}")


def initial_contact_audit(env) -> str:
    """Reject a start pose whose geoms already interpenetrate at t=0.

    A penetrating start pose settles during warmup, so every later physics,
    render, and task assertion still passes; this is the only check that sees it.
    Both the compiled ``qpos0`` and the post-reset state are audited because a
    reset that assigns positions can reintroduce overlap the MJCF does not have.
    """
    for label, data in (("model_qpos0", mujoco.MjData(env.model)), ("post_reset", env.data)):
        mujoco.mj_forward(env.model, data)
        for index in range(data.ncon):
            contact = data.contact[index]
            if contact.dist >= -1e-4:
                continue
            first = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            second = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            raise AssertionError(
                f"{label}: {first} and {second} interpenetrate by "
                f"{-contact.dist * 1000:.3f} mm before the first step"
            )
    return "PASS"


class SequenceContactAudit:
    """Watch every physics step of the documented sequence for interpenetration.

    ``initial_contact_audit`` describes the start pose and the endpoint
    tolerances below describe the ends of each motion. Neither sees the middle of
    a transfer, where a centimetre-deep overlap satisfies both. The module-level
    ``mujoco.mj_step`` is wrapped rather than an environment method because the
    settle loops and the generated controllers call the module function directly.

    Sub-millimetre readings are the solver's contact softness under load, so the
    limit is ``-1e-3``; anything deeper is geometry passing through geometry.
    """

    LIMIT = -1e-3

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.steps = 0
        self.worst = 0.0
        self.pair: str | None = None
        self.time_s = 0.0
        self._genuine = mujoco.mj_step

    def __enter__(self) -> "SequenceContactAudit":
        def watched(model, data, *args, **kwargs):
            self._genuine(model, data, *args, **kwargs)
            self.steps += 1
            for index in range(data.ncon):
                contact = data.contact[index]
                if contact.dist < self.worst:
                    self.worst = float(contact.dist)
                    self.pair = "{} / {}".format(
                        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
                        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
                    )
                    self.time_s = float(data.time)

        mujoco.mj_step = watched
        return self

    def __exit__(self, *exc_info: Any) -> None:
        mujoco.mj_step = self._genuine

    def report(self) -> dict[str, Any]:
        if self.worst < self.LIMIT:
            raise AssertionError(
                f"{self.pair} interpenetrate by {-self.worst * 1000:.3f} mm "
                f"at t={self.time_s:.3f} s during the documented sequence"
            )
        return {
            "steps": self.steps,
            "deepest_overlap_mm": round(max(0.0, -self.worst) * 1000.0, 4),
            "limit_mm": 1.0,
            "result": "PASS",
        }


def run(args: argparse.Namespace) -> dict[str, Any]:
    if os.environ.get("MUJOCO_GL") != "disable":
        raise AssertionError("physics_smoke.py must run with MUJOCO_GL=disable")
    ET.parse(args.model)
    env: ConveyorArmEnvironment = build_environment(args.model, args.spec)
    initial_contact_audit(env)
    if xyzw_to_wxyz([0, 0, 0, 1]) != [1.0, 0.0, 0.0, 0.0]:
        raise AssertionError("quaternion conversion failed")
    for name in ("arm_shoulder", "arm_elbow", "arm_wrist", "tool_z", "gripper_left_slide", "gripper_right_slide", "conveyor_drive_hinge", "parcel_free"):
        env.require_id("joint", name)
    for name in ("conveyor_motor", "shoulder_motor", "elbow_motor", "wrist_motor", "tool_lift_motor", "gripper_left_motor", "gripper_right_motor"):
        env.require_id("actuator", name)
    if env.model.jnt_type[env.conveyor_joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
        raise AssertionError("conveyor drive must be a hinge joint")
    if env.model.jnt_type[env.parcel_joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        raise AssertionError("parcel must be a free joint")
    initial = env.observe()
    if not all(math.isfinite(value) for value in initial["parcel_position"] + initial["arm_joint_angles_rad"]):
        raise AssertionError("initial state is not finite")
    expect_error(lambda: env.step({"id": "move_arm_to_parcel", "payload": {}}), "dependency enforcement")
    expect_error(lambda: env.step({"id": "unknown", "payload": {}}), "unknown interaction")
    expect_error(lambda: env.step({"id": "start_conveyor_to_pickup", "payload": {"speed_mps": 0.1}}), "speed bounds")
    expect_error(lambda: env.reset(seed=True), "boolean seed")

    with SequenceContactAudit(env.model) as audit:
        env.step({"id": "start_conveyor_to_pickup", "payload": {"speed_mps": 0.6}})
        if env.state["parcel_state"] != "at_pickup" or float(env.parcel_position[0]) < -0.11:
            raise AssertionError("conveyor did not deliver parcel")
        env.step({"id": "move_arm_to_parcel", "payload": {"speed_rad_s": 1.0}})
        if env.state["arm_state"] != "at_parcel":
            raise AssertionError("arm did not reach parcel")
        expect_error(lambda: env.step({"id": "grasp_parcel_with_arm", "payload": {"close": False}}), "grasp boolean")
        env.step({"id": "grasp_parcel_with_arm", "payload": {"close": True}})
        before_transport = np.asarray(env.observe()["parcel_position"])
        env.step({"id": "move_arm_to_target_bin", "payload": {"speed_rad_s": 1.0}})
        after_transport = np.asarray(env.observe()["parcel_position"])
        if float(np.linalg.norm(after_transport - before_transport)) < 0.06:
            raise AssertionError("held parcel did not follow arm")
        expect_error(lambda: env.step({"id": "release_parcel_in_target_bin", "payload": {"open": False}}), "release boolean")
        env.step({"id": "release_parcel_in_target_bin", "payload": {"open": True}})
        if env.state["parcel_state"] != "inside_target_bin" or not env.observe()["parcel_inside_target_bin"]:
            raise AssertionError("parcel did not settle in target bin")
    sequence_contact = audit.report()
    if env.is_success():
        raise AssertionError("inspection is required before success")
    env.reset(seed=808)
    if env.observe()["seed"] != 808 or env.observe()["state"]["history"]:
        raise AssertionError("custom reset failed")
    env.reset()
    return {"status": "PASS", "mujoco_executed": True, "mujoco_version": mujoco.__version__, "path_base": "package_root", "static_checks": {"xml_parse": "PASS", "mjcf_compile": "PASS", "conveyor_hinge": "PASS", "parcel_free_joint": "PASS", "explicit_actuators": 7, "visible_markers": len(env.points)}, "physics": {"initial_contact": "PASS", "mj_step": "PASS", "finite_state": "PASS", "conveyor_delivery": "PASS", "held_parcel_transport": "PASS", "target_bin_settle": "PASS", "sequence_contact": sequence_contact}, "dependency_enforcement": "PASS", "deterministic_reset": "PASS", "interaction_sequence": ["start_conveyor_to_pickup", "move_arm_to_parcel", "grasp_parcel_with_arm", "move_arm_to_target_bin", "release_parcel_in_target_bin"]}


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--result", type=Path, default=base / "output" / "physics_results.json")
    args = parser.parse_args(); args.model = args.model.resolve(); args.spec = args.spec.resolve(); args.result = args.result.resolve()
    try:
        result = run(args); exit_code = 0
    except Exception as exc:
        result = {"status": "FAIL", "mujoco_executed": True, "mujoco_version": getattr(mujoco, "__version__", "unknown"), "path_base": "package_root", "error_type": type(exc).__name__, "error": type(exc).__name__}; exit_code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True); args.result.write_text(json.dumps(result, indent=2), encoding="utf-8"); print(json.dumps(result, indent=2)); return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
