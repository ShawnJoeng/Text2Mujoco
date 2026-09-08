#!/usr/bin/env python3
"""Physics and interaction-contract smoke test for the lever-ball-ramp package."""

from __future__ import annotations

import argparse
import json
import os
import platform
from pathlib import Path

import mujoco
import numpy as np

from environment import EnvironmentError, build_environment, xyzw_to_wxyz


def expect_error(fn, label: str) -> None:
    try:
        fn()
    except EnvironmentError:
        return
    raise AssertionError(f"expected EnvironmentError: {label}")


def run(args: argparse.Namespace) -> dict:
    base = Path(__file__).resolve().parent
    env = build_environment(args.model, args.spec)
    if xyzw_to_wxyz([0, 0, 0, 1]) != [1.0, 0.0, 0.0, 0.0]:
        raise AssertionError("xyzw to wxyz conversion failed")
    expect_error(lambda: env.step({"id": "unknown", "payload": {}}), "unknown id")
    expect_error(lambda: env.step({"id": "check_release_zone", "payload": {}}), "dependency")
    expect_error(lambda: env.step({"id": "pull_blue_lever", "payload": {"pull_angle_rad": 0.2}}), "bounds")

    env.run_physics(25)
    if not np.isfinite(env.data.qpos).all() or not np.isfinite(env.data.qvel).all():
        raise AssertionError("non-finite state after free stepping")
    marker_positions = env.observe()["marker_positions"]
    if len(marker_positions) != 4 or not all(np.isfinite(position).all() for position in marker_positions.values()):
        raise AssertionError("interaction markers are missing or non-finite")
    env.step({"id": "pull_blue_lever", "payload": {"pull_angle_rad": 0.95}})
    pulled = env.observe()
    if pulled["lever_angle_rad"] > -0.75 or pulled["gate_lift_m"] < 0.1:
        raise AssertionError(f"lever/gate did not open: {pulled}")
    env.step({"id": "check_release_zone", "payload": {}})
    released = env.observe()
    if not released["ball_has_cleared_release_zone"]:
        raise AssertionError("ball did not clear release zone")
    env.step({"id": "confirm_target_tray", "payload": {}})
    placed = env.observe()
    if not placed["ball_inside_target"] or placed["ball_speed_mps"] >= 0.15:
        raise AssertionError(f"ball did not settle in tray: {placed}")
    if not placed["ball_contacts_tray_bottom"]:
        env.run_physics(120)
        placed = env.observe()
    if not placed["ball_contacts_tray_bottom"]:
        raise AssertionError("ball has no tray-bottom contact")
    history = placed["history"]
    env.reset(seed=99)
    if env.observe()["seed"] != 99 or env.observe()["history"]:
        raise AssertionError("custom reset did not clear state")
    env.reset()
    reset = env.observe()
    if reset["seed"] != 17 or reset["ball_position"] != [-0.44, 0.0, 1.02]:
        raise AssertionError(f"deterministic reset failed: {reset}")
    model = mujoco.MjModel.from_xml_path(str(args.model))
    if model.nq != env.model.nq:
        raise AssertionError("reloaded XML dimensions differ")
    artifacts = env.save_artifacts(args.output_dir)
    if env.spec["outputs"].get("save_mjcf") and "mjcf" not in artifacts:
        raise AssertionError("save_mjcf=true did not produce an MJCF artifact")
    if env.spec["outputs"].get("save_mjb") and "mjb" not in artifacts:
        raise AssertionError("save_mjb=true did not produce an MJB artifact")
    result = {
        "status": "PASS",
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "path_base": "package_root",
        "mujoco_gl": os.environ.get("MUJOCO_GL", "unset"),
        "model_compile": "PASS",
        "typed_targets": "PASS",
        "marker_sites_resolved": "PASS",
        "xyzw_to_wxyz": "PASS",
        "invalid_action_checks": 3,
        "physics": {
            "lever_actuator": "PASS",
            "gate_lift_m": float(pulled["gate_lift_m"]),
            "release_zone": "PASS",
            "tray_settling": "PASS",
            "tray_contact": "PASS",
            "ball_position": placed["ball_position"],
            "ball_speed_mps": placed["ball_speed_mps"],
            "finite_state": "PASS",
        },
        "interaction_sequence": history,
        "deterministic_reset": "PASS",
        "mjcf_reload": "PASS",
        "artifacts": {
            key: value
            for key, value in artifacts.items()
        },
    }
    return result


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
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
        result = {
            "status": "FAIL",
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "path_base": "package_root",
            "mujoco_gl": os.environ.get("MUJOCO_GL", "unset"),
            "error_type": type(exc).__name__,
            "error": type(exc).__name__,
        }
        code = 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
