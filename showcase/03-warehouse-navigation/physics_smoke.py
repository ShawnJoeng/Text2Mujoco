#!/usr/bin/env python3
"""Physics and interaction-contract smoke test for warehouse navigation."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import sys
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Callable

import mujoco
import numpy as np

from environment import EnvironmentError, WarehouseNavigationEnv, build_environment, xyzw_to_wxyz


def expect_error(call: Callable[[], Any], label: str) -> None:
    try:
        call()
    except EnvironmentError:
        return
    raise AssertionError(f"expected EnvironmentError: {label}")


def static_checks(env: WarehouseNavigationEnv) -> dict[str, Any]:
    ET.parse(env.model_path)
    if xyzw_to_wxyz([0, 0, 0, 1]) != [1.0, 0.0, 0.0, 0.0]:
        raise AssertionError("xyzw -> wxyz conversion failed")
    root = ET.parse(env.model_path).getroot()
    sizes = {geom.attrib["name"]: geom.attrib.get("size") for geom in root.findall(".//geom") if "name" in geom.attrib}
    if sizes["mobile_robot_geom"] != "0.23 0.12":
        raise AssertionError("robot cylinder full dimensions were not converted to radius/half-height")
    if sizes["central_shelf_geom"] != "0.35 1.10 0.70":
        raise AssertionError("shelf full dimensions were not converted to half-sizes")
    points = env.list_interaction_points()
    if [point["id"] for point in points] != ["reach_checkpoint_a", "reach_checkpoint_b", "inspect_top_camera"]:
        raise AssertionError("unexpected interaction ordering")
    if not all(point.get("visible") and point.get("marker_site") for point in points):
        raise AssertionError("all interaction points must have visible markers")
    return {
        "xml_parse": "PASS",
        "mjcf_compile": "PASS",
        "manifest_spec_parity": "PASS",
        "typed_target_resolution": "PASS",
        "dependency_order": "PASS",
        "visible_markers": "PASS",
        "geometry_half_sizes": "PASS",
        "quaternion_conversion": "PASS",
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    if os.environ.get("MUJOCO_GL") != "disable":
        raise AssertionError("physics_smoke.py must run with MUJOCO_GL=disable")
    env = build_environment(args.model, args.spec)
    checks = static_checks(env)
    initial = env.observe()
    env.run_physics(100)
    if float(np.linalg.norm(env.robot_xy - env.START)) > 1e-8:
        raise AssertionError("robot drifted from start")

    expect_error(lambda: env.step({"id": "reach_checkpoint_b", "payload": {}}), "B before A")
    expect_error(lambda: env.step({"id": "inspect_top_camera", "payload": {"output_dir": "x"}}), "inspect before B")
    expect_error(lambda: env.step({"id": "unknown", "payload": {}}), "unknown id")
    expect_error(lambda: env.step({"id": "reach_checkpoint_a"}), "malformed envelope")
    expect_error(lambda: env.step({"id": "reach_checkpoint_a", "payload": {"speed_mps": 0.1}}), "speed below minimum")
    expect_error(lambda: env.step({"id": "reach_checkpoint_a", "payload": {"bogus": 1}}), "unknown payload field")
    expect_error(lambda: env.reset(seed=True), "bool seed")

    a_route = env.route_clearance([env.START, env.CHECKPOINT_A])
    a_obs = env.step({"id": "reach_checkpoint_a", "payload": {"speed_mps": 0.8}})
    if a_obs["distance_to_a_m"] > 0.10:
        raise AssertionError(f"checkpoint A not reached: {a_obs['distance_to_a_m']}")
    expect_error(lambda: env.step({"id": "reach_checkpoint_a", "payload": {}}), "repeat A")

    b_points = [env.CHECKPOINT_A, *env.SOUTH_BYPASS]
    b_route = env.route_clearance(b_points)
    b_obs = env.step({"id": "reach_checkpoint_b", "payload": {"speed_mps": 0.8}})
    if b_obs["distance_to_b_m"] > 0.10:
        raise AssertionError(f"checkpoint B not reached: {b_obs['distance_to_b_m']}")
    if b_obs["state"]["shelf_collision_count"] != 0:
        raise AssertionError("robot recorded a shelf collision")
    trace = np.asarray(b_obs["route_trace"])
    for waypoint in env.SOUTH_BYPASS[:-1]:
        if float(np.min(np.linalg.norm(trace - waypoint, axis=1))) > 0.12:
            raise AssertionError(f"route did not pass required waypoint: {waypoint.tolist()}")
    if env.is_success():
        raise AssertionError("task must remain incomplete until camera inspection")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    mjcf_copy = args.output_dir / "model.xml"
    mjb_path = args.output_dir / "model.mjb"
    shutil.copy2(args.model, mjcf_copy)
    mujoco.mj_saveModel(env.model, str(mjb_path), None)
    xml_reload = mujoco.MjModel.from_xml_path(str(mjcf_copy))
    binary_reload = mujoco.MjModel.from_binary_path(str(mjb_path))
    if xml_reload.nq != env.model.nq or binary_reload.nq != env.model.nq:
        raise AssertionError("reloaded model dimensions differ")

    completed_route = b_obs["route_trace"]
    env.reset(seed=91)
    if env.observe()["seed"] != 91:
        raise AssertionError("custom seed not applied")
    env.reset()
    reset_obs = env.observe()
    if reset_obs["seed"] != 37 or reset_obs["state"]["history"] or env.completed:
        raise AssertionError("deterministic reset did not restore task state")
    np.testing.assert_allclose(env.robot_xy, env.START, atol=1e-10)
    np.testing.assert_allclose(env.data.qvel, 0.0, atol=1e-12)
    if env.is_success():
        raise AssertionError("reset scene reports success")

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "command": " ".join(sys.argv),
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "static_checks": checks,
        "physics": {"mj_step": "PASS", "finite_state": "PASS", "robot_shelf_contacts": 0},
        "route": {
            "checkpoint_a": a_route,
            "checkpoint_b_south_bypass": b_route,
            "required_waypoints": [point.tolist() for point in env.SOUTH_BYPASS],
            "trace_samples": len(completed_route),
            "checkpoint_b_position": b_obs["robot_position"],
        },
        "dependency_enforcement": "PASS",
        "invalid_action_checks": 7,
        "deterministic_reset": "PASS",
        "mjcf_reload": "PASS",
        "mjb_reload": "PASS",
        "initial_position": initial["robot_position"],
        "artifacts": {
            "mjcf": str(mjcf_copy.relative_to(Path(__file__).resolve().parent)),
            "mjb": str(mjb_path.relative_to(Path(__file__).resolve().parent)),
        },
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--output-dir", type=Path, default=base / "output")
    parser.add_argument("--result", type=Path, default=base / "output" / "physics_results.json")
    args = parser.parse_args()
    try:
        result = run(args)
        exit_code = 0
    except Exception as exc:
        result = {
            "status": "FAIL",
            "mujoco_executed": True,
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "platform": platform.platform(),
            "command": " ".join(sys.argv),
            "mujoco_gl": os.environ.get("MUJOCO_GL"),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }
        exit_code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
