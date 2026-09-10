#!/usr/bin/env python3
"""Physics and interaction-contract smoke test for robot peg assembly."""

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

from environment import EnvironmentError, RobotAssemblyEnvironment, build_environment, xyzw_to_wxyz


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


def run(args: argparse.Namespace) -> dict[str, Any]:
    if os.environ.get("MUJOCO_GL") != "disable":
        raise AssertionError("physics_smoke.py must run with MUJOCO_GL=disable")
    ET.parse(args.model)
    env: RobotAssemblyEnvironment = build_environment(args.model, args.spec)
    initial_contact_audit(env)
    if xyzw_to_wxyz([0, 0, 0, 1]) != [1.0, 0.0, 0.0, 0.0]:
        raise AssertionError("quaternion conversion failed")
    for name in ("arm_shoulder", "arm_elbow", "arm_wrist", "tool_z", "gripper_slide", "peg_free"):
        env.require_id("joint", name)
    for name in ("shoulder_motor", "elbow_motor", "wrist_motor", "tool_lift_motor", "gripper_motor"):
        env.require_id("actuator", name)
    if any(env.model.jnt_type[env.require_id("joint", name)] != mujoco.mjtJoint.mjJNT_HINGE for name in ("arm_shoulder", "arm_elbow", "arm_wrist")):
        raise AssertionError("arm joints must be hinge joints")
    if env.model.jnt_type[env.tool_joint_id] != mujoco.mjtJoint.mjJNT_SLIDE:
        raise AssertionError("tool_z must be a slide joint")
    if env.model.jnt_type[env.peg_joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        raise AssertionError("peg_free must be a free joint")
    if not all(point.get("marker_site") for point in env.list_interaction_points()):
        raise AssertionError("every interaction must expose a visible marker")

    env.run_physics(100)
    initial = env.observe()
    if not all(math.isfinite(value) for value in initial["arm_joint_qpos"] + initial["peg_position"]):
        raise AssertionError("initial state is not finite")
    expect_error(lambda: env.step({"id": "move_arm_to_socket", "payload": {}}), "dependency enforcement")
    expect_error(lambda: env.step({"id": "unknown", "payload": {}}), "unknown interaction")
    expect_error(lambda: env.step({"id": "move_arm_to_peg", "payload": {"speed_rad_s": 0.1}}), "speed bounds")
    expect_error(lambda: env.step({"id": "move_arm_to_peg", "payload": {"bogus": True}}), "unknown payload")
    expect_error(lambda: env.reset(seed=True), "boolean seed")

    env.step({"id": "move_arm_to_peg", "payload": {"speed_rad_s": 1.0}})
    if env.state["arm_state"] != "at_peg":
        raise AssertionError("arm did not reach peg")
    expect_error(lambda: env.step({"id": "grasp_peg_with_arm", "payload": {"close": False}}), "grasp boolean")
    env.step({"id": "grasp_peg_with_arm", "payload": {"close": True}})
    held_before = np.asarray(env.observe()["peg_position"])
    env.step({"id": "move_arm_to_socket", "payload": {"speed_rad_s": 1.0}})
    held_after = np.asarray(env.observe()["peg_position"])
    if float(np.linalg.norm(held_after - held_before)) < 0.05:
        raise AssertionError("held peg did not follow arm transport")
    env.step({"id": "insert_peg_into_socket", "payload": {"depth_m": 0.08}})
    if env.observe()["tool_z_qpos"] > -0.06:
        raise AssertionError("tool did not lower")
    expect_error(lambda: env.step({"id": "release_assembled_peg", "payload": {"open": False}}), "release boolean")
    env.step({"id": "release_assembled_peg", "payload": {"open": True}})
    if env.state["peg_state"] != "seated":
        raise AssertionError("peg did not seat")
    peg_error = float(np.linalg.norm(np.asarray(env.observe()["peg_position"]) - env.SOCKET_POSITION))
    if peg_error > 0.075:
        raise AssertionError(f"peg is outside socket tolerance: {peg_error:.4f} m")
    if env.is_success():
        raise AssertionError("inspection is required before success")

    env.reset(seed=808)
    if env.observe()["seed"] != 808 or env.observe()["state"]["history"]:
        raise AssertionError("custom reset did not clear state")
    env.reset()
    reset = env.observe()
    if reset["seed"] != 107 or reset["state"]["arm_state"] != "home":
        raise AssertionError("default reset did not restore initial state")

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "path_base": "package_root",
        "static_checks": {
            "xml_parse": "PASS",
            "mjcf_compile": "PASS",
            "quaternion_conversion": "PASS",
            "arm_hinge_joints": 3,
            "explicit_actuators": 5,
            "visible_markers": len(env.points),
        },
        "physics": {"initial_contact": "PASS", "mj_step": "PASS", "finite_state": "PASS", "held_peg_transport": "PASS", "socket_release_tolerance_m": peg_error},
        "dependency_enforcement": "PASS",
        "deterministic_reset": "PASS",
        "interaction_sequence": [
            "move_arm_to_peg",
            "grasp_peg_with_arm",
            "move_arm_to_socket",
            "insert_peg_into_socket",
            "release_assembled_peg",
        ],
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--result", type=Path, default=base / "output" / "physics_results.json")
    args = parser.parse_args()
    args.model = args.model.resolve()
    args.spec = args.spec.resolve()
    args.result = args.result.resolve()
    try:
        result = run(args)
        exit_code = 0
    except Exception as exc:
        result = {"status": "FAIL", "mujoco_executed": True, "mujoco_version": getattr(mujoco, "__version__", "unknown"), "path_base": "package_root", "error_type": type(exc).__name__, "error": type(exc).__name__}
        exit_code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
