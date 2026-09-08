#!/usr/bin/env python3
"""Static-contract and MuJoCo physics smoke test."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from environment import EnvironmentError, build_environment, xyzw_to_wxyz


def expect_error(call, label: str) -> None:
    try:
        call()
    except EnvironmentError:
        return
    raise AssertionError(f"expected EnvironmentError for {label}")


def run(args: argparse.Namespace) -> dict:
    if os.environ.get("MUJOCO_GL") != "disable":
        raise AssertionError("physics smoke must run with MUJOCO_GL=disable")
    ET.parse(args.model)
    environment = build_environment(args.model, args.spec)
    if mujoco.__version__ != "3.2.7":
        raise AssertionError(f"expected MuJoCo 3.2.7, got {mujoco.__version__}")
    if xyzw_to_wxyz([0, 0, 0, 1]) != [1.0, 0.0, 0.0, 0.0]:
        raise AssertionError("xyzw to wxyz conversion failed")

    points = environment.list_interaction_points()
    if [point["id"] for point in points] != [
        "press_unlock_button", "pull_drawer_22cm", "inspect_open_drawer"
    ]:
        raise AssertionError("interaction sequence mismatch")
    if any(not point.get("marker_site") for point in points):
        raise AssertionError("an interaction point is missing its visible marker")
    for marker in ("unlock_point_marker", "drawer_handle_marker", "camera_check_marker"):
        environment.require_id("site", marker)

    environment.run_physics(100)
    if not all(math.isfinite(value) for value in environment.data.qpos):
        raise AssertionError("initial simulation contains non-finite qpos")
    expect_error(lambda: environment.step({"id": "missing", "payload": {}}), "unknown id")
    expect_error(
        lambda: environment.step({"id": "pull_drawer_22cm", "payload": {"distance_m": 0.22}}),
        "locked drawer",
    )
    expect_error(
        lambda: environment.step({"id": "press_unlock_button", "payload": {"press_depth_m": 0.009}}),
        "short button press",
    )
    expect_error(
        lambda: environment.step({"id": "pull_drawer_22cm", "payload": {"distance_m": 0.21}}),
        "wrong pull distance",
    )
    expect_error(lambda: environment.run_physics(True), "boolean step count")

    environment.step({"id": "press_unlock_button", "payload": {"press_depth_m": 0.012}})
    pressed = environment.observe()
    if pressed["unlock_button_joint_qpos"] < 0.01 or pressed["state"]["lock_state"] != "unlocked":
        raise AssertionError("button did not physically unlock")
    environment.step({"id": "pull_drawer_22cm", "payload": {"distance_m": 0.22}})
    opened = environment.observe()
    if abs(opened["drawer_joint_qpos"] - 0.22) > 0.002:
        raise AssertionError(f"drawer target error: {opened['drawer_joint_qpos']}")
    if opened["state"]["history"] != ["press_unlock_button", "pull_drawer_22cm"]:
        raise AssertionError("interaction history mismatch")
    if environment.is_success():
        raise AssertionError("physics-only sequence must still require camera inspection")

    artifacts = environment.save_artifacts(args.output_dir)
    xml_reload = mujoco.MjModel.from_xml_path(artifacts["mjcf"])
    binary_reload = mujoco.MjModel.from_binary_path(artifacts["mjb"])
    if xml_reload.nq != environment.model.nq or binary_reload.nq != environment.model.nq:
        raise AssertionError("serialized model dimensions changed")

    for bad_seed in (True, -1, 1.5, "22"):
        expect_error(lambda value=bad_seed: environment.reset(value), "invalid seed")
    environment.reset(seed=99)
    if environment.observe()["seed"] != 99:
        raise AssertionError("explicit reset seed was ignored")
    environment.reset()
    reset = environment.observe()
    if reset["seed"] != 22 or reset["state"]["history"]:
        raise AssertionError("deterministic reset state mismatch")
    np.testing.assert_allclose(environment.data.qpos, 0.0, atol=1e-12)
    np.testing.assert_allclose(environment.data.qvel, 0.0, atol=1e-12)

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "command": " ".join(sys.argv),
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "mjcf_compile": "PASS",
        "finite_state": "PASS",
        "unlock_actuator": "PASS",
        "drawer_actuator": "PASS",
        "target_distance_m": 0.22,
        "measured_distance_m": opened["drawer_joint_qpos"],
        "visible_marker_names": ["unlock_point_marker", "drawer_handle_marker", "camera_check_marker"],
        "invalid_action_checks": 5,
        "deterministic_reset": "PASS",
        "mjcf_reload": "PASS",
        "mjb_reload": "PASS",
        "artifacts": {
            key: str(
                Path(value).resolve().relative_to(Path(__file__).resolve().parent)
            )
            for key, value in artifacts.items()
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
        result, exit_code = run(args), 0
    except Exception as exc:
        result, exit_code = {
            "status": "FAIL",
            "mujoco_executed": True,
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "command": " ".join(sys.argv),
            "mujoco_gl": os.environ.get("MUJOCO_GL"),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }, 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
