#!/usr/bin/env python3
"""Physics and interaction smoke test for the robotic arm sorting cell."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Callable

import mujoco
import numpy as np

from environment import EnvironmentError, build_environment, xyzw_to_wxyz


def expect_error(callback: Callable[[], Any], label: str) -> None:
    try:
        callback()
    except EnvironmentError:
        return
    raise AssertionError(f"expected EnvironmentError: {label}")


def static_checks(env) -> dict[str, Any]:
    ET.parse(env.model_path)
    names = {
        "joint": ["arm_shoulder", "arm_elbow", "arm_wrist", "gripper_left_slide", "gripper_right_slide", "blue_part_free"],
        "actuator": ["arm_shoulder_motor", "arm_elbow_motor", "arm_wrist_motor", "gripper_left_motor", "gripper_right_motor"],
        "body": ["arm_base", "arm_upper_link", "arm_forearm_link", "arm_wrist_link", "gripper", "blue_part", "blue_bin"],
        "site": [point["marker_site"] for point in env.spec["interaction_points"]] + ["arm_tcp"],
    }
    for object_type, values in names.items():
        for value in values:
            env.require_id(object_type, value)
    if env.model.nq < 19 or env.model.nu < 5:
        raise AssertionError("articulated arm model has too few degrees of freedom/actuators")
    if xyzw_to_wxyz([0, 0, 0, 1]) != [1.0, 0.0, 0.0, 0.0]:
        raise AssertionError("quaternion conversion failed")
    points = env.list_interaction_points()
    expected_order = [
        "approach_blue_part",
        "grasp_blue_part",
        "transfer_to_blue_bin",
        "release_blue_part",
        "inspect_sorting_result",
    ]
    if [point["id"] for point in points] != expected_order:
        raise AssertionError("unexpected interaction ordering")
    if not all(point.get("marker_site") for point in points):
        raise AssertionError("all interaction points must declare marker sites")
    return {
        "xml_parse": "PASS",
        "mjcf_compile": "PASS",
        "articulated_joints": 5,
        "arm_actuators": 5,
        "typed_targets": "PASS",
        "visible_markers": "PASS",
        "quaternion_conversion": "PASS",
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    if os.environ.get("MUJOCO_GL") != "disable":
        raise AssertionError("physics_smoke.py must run with MUJOCO_GL=disable")
    env = build_environment(args.model, args.spec)
    checks = static_checks(env)
    initial = env.observe()
    if initial["state"]["arm_state"] != "home" or initial["state"]["gripper_state"] != "open":
        raise AssertionError("reset did not initialize robot state")
    expect_error(lambda: env.step({"id": "grasp_blue_part", "payload": {"close": True}}), "grasp before approach")
    expect_error(lambda: env.step({"id": "approach_blue_part", "payload": {"speed_rad_s": 0.1}}), "speed lower bound")
    expect_error(lambda: env.step({"id": "approach_blue_part", "payload": {"unknown": 1}}), "unknown payload")
    expect_error(lambda: env.step({"id": "missing", "payload": {}}), "unknown interaction")
    expect_error(lambda: env.reset(seed=True), "boolean seed")
    env.run_physics(40)
    if not np.all(np.isfinite(env.data.qpos)):
        raise AssertionError("state became non-finite during free physics")

    approach = env.step({"id": "approach_blue_part", "payload": {"speed_rad_s": 1.0}})
    if approach["state"]["arm_state"] != "at_pick_pose":
        raise AssertionError("arm did not enter pick pose")
    if float(np.linalg.norm(np.asarray(approach["arm_tcp_position"]) - env.BLUE_PICK)) > 0.11:
        raise AssertionError("TCP is not aligned with blue part")

    grasp = env.step({"id": "grasp_blue_part", "payload": {"close": True}})
    if grasp["state"]["blue_part_state"] != "grasped" or grasp["state"]["gripper_state"] != "closed":
        raise AssertionError("gripper did not grasp the blue part")
    held_before = np.asarray(grasp["blue_part_position"], dtype=float)
    transfer = env.step({"id": "transfer_to_blue_bin", "payload": {"speed_rad_s": 1.0}})
    if transfer["state"]["arm_state"] != "over_blue_bin":
        raise AssertionError("arm did not reach blue bin")
    held_after = np.asarray(transfer["blue_part_position"], dtype=float)
    if np.linalg.norm(held_after - held_before) < 0.20:
        raise AssertionError("grasped object did not follow the arm during transfer")
    if np.linalg.norm(held_after - np.asarray(transfer["arm_tcp_position"], dtype=float)) > 0.08:
        raise AssertionError("grasped object is not synchronized with arm TCP")
    expect_error(lambda: env.step({"id": "release_blue_part", "payload": {"open": False}}), "release flag")
    released = env.step({"id": "release_blue_part", "payload": {"open": True}})
    if released["state"]["blue_part_state"] != "inside_blue_bin" or not released["blue_part_inside_bin"]:
        raise AssertionError("released blue part is not inside the target bin")
    if env.is_success():
        raise AssertionError("inspection must be required for success")

    artifact_root = env.package_root / "output"
    artifact_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="text2mujoco-arm-", dir=artifact_root) as temp_dir:
        artifacts = env.save_artifacts(Path(temp_dir))
        if set(artifacts) != {"mjcf", "mjb"}:
            raise AssertionError("artifact output flags did not produce both artifacts")
        xml_reload = mujoco.MjModel.from_xml_path(str(env.package_root / artifacts["mjcf"]))
        binary_reload = mujoco.MjModel.from_binary_path(str(env.package_root / artifacts["mjb"]))
        if xml_reload.nq != env.model.nq or binary_reload.nq != env.model.nq:
            raise AssertionError("reloaded model dimensions differ")

    env.reset(seed=91)
    if env.observe()["seed"] != 91:
        raise AssertionError("custom seed was not applied")
    env.reset()
    reset = env.observe()
    if reset["history"] or reset["state"]["blue_part_state"] != "on_conveyor":
        raise AssertionError("reset did not restore task state")
    if not np.allclose(env.arm_angles, env.HOME_ANGLES, atol=1e-8):
        raise AssertionError("reset did not restore arm home pose")

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "path_base": "package_root",
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "static_checks": checks,
        "physics": {"mj_step": "PASS", "finite_state": "PASS"},
        "interaction_sequence": ["approach_blue_part", "grasp_blue_part", "transfer_to_blue_bin", "release_blue_part"],
        "grasp_sync": "PASS",
        "release_inside_bin": "PASS",
        "dependency_enforcement": "PASS",
        "deterministic_reset": "PASS",
        "mjcf_reload": "PASS",
        "mjb_reload": "PASS",
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--output-dir", type=Path, default=base / "output")
    parser.add_argument("--result", type=Path, default=base / "output" / "physics_results.json")
    args = parser.parse_args()
    args.model = args.model.resolve()
    args.spec = args.spec.resolve()
    args.output_dir = args.output_dir.resolve()
    args.result = args.result.resolve()
    try:
        result = run(args)
        code = 0
    except Exception as exc:
        result = {"status": "FAIL", "mujoco_executed": True, "mujoco_version": mujoco.__version__, "path_base": "package_root", "error_type": type(exc).__name__, "error": type(exc).__name__}
        code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
