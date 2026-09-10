#!/usr/bin/env python3
"""End-to-end MuJoCo physics and interaction smoke test."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import tempfile
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from environment import EnvironmentError, build_environment, xyzw_to_wxyz


def expect_environment_error(callback, label: str) -> None:
    try:
        callback()
    except EnvironmentError:
        return
    raise AssertionError(f"{label}: expected EnvironmentError")


def check_model(environment) -> dict[str, Any]:
    model = environment.model
    assets = environment._assets
    expected = {
        "body": [asset["body_name"] for asset in assets.values()],
        "geom": [
            name
            for asset in assets.values()
            for name in (
                asset["geometry"].get("part_geom_names", [])
                if asset["geometry"].get("shape") == "open_box"
                else [asset["geometry"].get("geom_name")]
            )
            if name
        ],
        "joint": [
            asset["physics"]["joint_name"]
            for asset in assets.values()
            if asset["physics"].get("joint_name")
        ],
        "actuator": [
            asset["physics"]["actuator_name"]
            for asset in assets.values()
            if asset["physics"].get("actuator_name")
        ],
        "camera": [
            sensor["camera_name"]
            for sensor in environment.spec["sensors"]
            if sensor.get("type") == "camera"
        ],
        "site": [
            point["marker_site"]
            for point in environment.spec["interaction_points"]
            if point.get("marker_site")
        ],
    }
    for object_type, names in expected.items():
        for name in names:
            environment.require_id(object_type, name)

    np.testing.assert_allclose(model.opt.gravity, [0.0, 0.0, -9.81], atol=1e-9)
    if not math.isclose(float(model.opt.timestep), 0.002, abs_tol=1e-12):
        raise AssertionError(f"unexpected timestep: {model.opt.timestep}")
    button_type = model.jnt_type[environment.button_joint_id]
    cube_type = model.jnt_type[environment.cube_joint_id]
    if int(button_type) != int(mujoco.mjtJoint.mjJNT_SLIDE):
        raise AssertionError("button_slide is not a slide joint")
    if int(cube_type) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise AssertionError("cube_free is not a free joint")
    if not math.isclose(float(model.body_mass[environment.cube_body_id]), 0.2, rel_tol=1e-6):
        raise AssertionError("red cube mass does not match the scene specification")
    if xyzw_to_wxyz([0, 0, 0, 1]) != [1.0, 0.0, 0.0, 0.0]:
        raise AssertionError("xyzw to wxyz quaternion conversion is incorrect")
    for point in environment.spec["interaction_points"]:
        marker_site = point.get("marker_site")
        if not marker_site:
            continue
        site_id = environment.require_id("site", marker_site)
        np.testing.assert_allclose(
            environment.data.site_xpos[site_id],
            point["pose"]["position"],
            atol=1e-9,
        )
    camera_spec = next(
        sensor for sensor in environment.spec["sensors"] if sensor.get("type") == "camera"
    )
    np.testing.assert_allclose(
        model.cam_pos[environment.camera_id], camera_spec["pose"]["position"], atol=1e-9
    )
    np.testing.assert_allclose(
        model.cam_quat[environment.camera_id],
        xyzw_to_wxyz(camera_spec["pose"]["orientation_xyzw"]),
        atol=1e-9,
    )
    return {
        "named_objects": sum(len(names) for names in expected.values()),
        "open_box_colliders": len(assets["target_box"]["geometry"]["part_geom_names"]),
        "gravity": [float(value) for value in model.opt.gravity],
        "timestep_s": float(model.opt.timestep),
        "cube_mass_kg": float(model.body_mass[environment.cube_body_id]),
        "quaternion_conversion": "PASS",
        "declared_pose_consistency": "PASS",
    }


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
        raise AssertionError("physics_smoke.py must run in a MUJOCO_GL=disable process")
    environment = build_environment(args.model, args.spec)
    initial_contact_audit(environment)
    model_checks = check_model(environment)
    for invalid_steps in (True, 1.5, "2", -1):
        expect_environment_error(
            lambda value=invalid_steps: environment.run_physics(value),
            "step count validation",
        )
    for method in (
        "list_interaction_points",
        "get_action_schema",
        "reset",
        "step",
        "observe",
        "is_success",
    ):
        if not callable(getattr(environment, method, None)):
            raise AssertionError(f"missing public environment method: {method}")

    environment.run_physics(100)
    initial_observation = environment.observe()
    if not all(math.isfinite(value) for value in initial_observation["cube_position"]):
        raise AssertionError("initial physics produced a non-finite cube pose")

    environment.reset()
    expect_environment_error(
        lambda: environment.step({"id": "grasp_red_cube", "payload": {"close": True}}),
        "dependency enforcement",
    )
    expect_environment_error(
        lambda: environment.step({"id": "missing", "payload": {}}),
        "unknown interaction",
    )
    for invalid_id in ([], {}, {"nested": "id"}):
        expect_environment_error(
            lambda value=invalid_id: environment.step({"id": value, "payload": {}}),
            "unhashable/invalid interaction id",
        )
    expect_environment_error(
        lambda: environment.step(
            {"id": "press_start_button", "payload": {"press_depth_m": 0.01}}
        ),
        "numeric bound validation",
    )
    expect_environment_error(
        lambda: environment.step(
            {"id": "press_start_button", "payload": {"press_depth_m": 0.0}}
        ),
        "insufficient press force",
    )
    expect_environment_error(
        lambda: environment.step(
            {"id": "press_start_button", "payload": {"press_depth_m": 0.021}}
        ),
        "maximum press depth validation",
    )

    environment.step({"id": "press_start_button", "payload": {"press_depth_m": 0.018}})
    pressed_qpos = environment.observe()["button_joint_qpos"]
    if pressed_qpos > -0.014:
        raise AssertionError(f"button did not press: qpos={pressed_qpos}")
    expect_environment_error(
        lambda: environment.step(
            {"id": "press_start_button", "payload": {"press_depth_m": 0.018}}
        ),
        "state precondition enforcement",
    )
    expect_environment_error(
        lambda: environment.step({"id": "grasp_red_cube", "payload": {"close": False}}),
        "boolean payload validation",
    )
    environment.step({"id": "grasp_red_cube", "payload": {"close": True}})
    held_position = np.asarray(environment.observe()["cube_position"])
    environment.run_physics(50)
    np.testing.assert_allclose(
        environment.observe()["cube_position"], held_position, atol=1e-9
    )
    expect_environment_error(
        lambda: environment.step(
            {
                "id": "place_cube_in_box",
                "payload": {"pose": [9.0, 9.0, 9.0], "release": True},
            }
        ),
        "target bound enforcement",
    )
    environment.step(
        {
            "id": "place_cube_in_box",
            "payload": {"pose": [0.3, 0.18, 0.86], "release": True},
        }
    )
    placed = environment.observe()
    if not environment.cube_is_inside_box():
        raise AssertionError(f"cube is outside box after release: {placed['cube_position']}")
    if not environment.cube_contacts_box_bottom():
        raise AssertionError("cube has no contact with the open-box bottom")
    if placed["cube_linear_speed_mps"] >= 0.1:
        raise AssertionError(f"cube did not settle: {placed['cube_linear_speed_mps']}")

    artifacts = environment.save_artifacts(args.output_dir)
    if environment.spec["outputs"].get("save_mjcf", False):
        xml_model = mujoco.MjModel.from_xml_path(
            str(environment.package_root / artifacts["mjcf"])
        )
        if xml_model.nq != environment.model.nq:
            raise AssertionError("reloaded MJCF dimensions differ from compiled model")
    if environment.spec["outputs"].get("save_mjb", False):
        binary_model = mujoco.MjModel.from_binary_path(
            str(environment.package_root / artifacts["mjb"])
        )
        if binary_model.nq != environment.model.nq:
            raise AssertionError("reloaded MJB dimensions differ from compiled model")
    original_outputs = environment.spec["outputs"]
    try:
        environment.spec["outputs"] = {
            "save_mjcf": False,
            "save_mjb": False,
        }
        with tempfile.TemporaryDirectory(prefix="text2mujoco-no-output-") as temp_dir:
            if environment.save_artifacts(Path(temp_dir)):
                raise AssertionError("disabled output flags still returned artifacts")
            if any(Path(temp_dir).iterdir()):
                raise AssertionError("disabled output flags still wrote artifacts")
    finally:
        environment.spec["outputs"] = original_outputs

    completed_history = placed["state"]["history"]
    for invalid_seed in (True, 1.9, "12", -1):
        expect_environment_error(
            lambda value=invalid_seed: environment.reset(seed=value),
            "reset seed validation",
        )
    environment.reset(seed=123)
    if environment.observe()["seed"] != 123:
        raise AssertionError("custom reset seed was not applied")
    environment.reset()
    reset_observation = environment.observe()
    if reset_observation["seed"] != 7:
        raise AssertionError("default reset seed was not restored")
    if reset_observation["state"]["button_state"] != "ready":
        raise AssertionError("reset did not restore button state")
    if reset_observation["state"]["cube_state"] != "free":
        raise AssertionError("reset did not restore cube state")
    if reset_observation["time_s"] != 0.0:
        raise AssertionError("reset did not restore simulation time")
    if abs(reset_observation["button_joint_qpos"]) > 1e-12:
        raise AssertionError("reset did not restore button qpos")
    np.testing.assert_allclose(
        reset_observation["cube_position"], [-0.12, -0.03, 0.825], atol=1e-9
    )
    np.testing.assert_allclose(environment.data.qvel, 0.0, atol=1e-12)
    np.testing.assert_allclose(environment.data.ctrl, 0.0, atol=1e-12)
    if reset_observation["state"]["history"] or environment.completed:
        raise AssertionError("reset did not clear interaction history/completion")
    if environment.is_success():
        raise AssertionError("physics-only/reset state must not report full RGB-D task success")

    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "path_base": "package_root",
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "model_checks": model_checks,
        "physics": {
            "initial_contact": "PASS",
            "mj_step": "PASS",
            "button_actuator": "PASS",
            "gravity_release": "PASS",
            "open_box_contact": "PASS",
            "finite_state": "PASS",
            "cube_position": placed["cube_position"],
            "cube_linear_speed_mps": placed["cube_linear_speed_mps"],
        },
        "interaction_sequence": completed_history,
        "invalid_action_checks": 11,
        "invalid_step_count_checks": 4,
        "invalid_reset_seed_checks": 4,
        "grasp_hold": "PASS",
        "deterministic_reset": "PASS",
        "mjcf_reload": "PASS" if environment.spec["outputs"].get("save_mjcf", False) else "SKIPPED",
        "mjb_reload": "PASS" if environment.spec["outputs"].get("save_mjb", False) else "SKIPPED",
        "output_flags": "PASS",
        "artifacts": {
            key: value
            for key, value in artifacts.items()
        },
    }


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    parser.add_argument("--spec", type=Path, default=base / "scene_spec.json")
    parser.add_argument("--output-dir", type=Path, default=base / "output")
    parser.add_argument(
        "--result", type=Path, default=base / "output" / "physics_results.json"
    )
    args = parser.parse_args()
    args.model = args.model.resolve()
    args.spec = args.spec.resolve()
    args.output_dir = args.output_dir.resolve()
    args.result = args.result.resolve()
    exit_code = 0
    try:
        result = run(args)
    except Exception as exc:
        exit_code = 1
        result = {
            "status": "FAIL",
            "mujoco_executed": True,
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "path_base": "package_root",
            "mujoco_gl": os.environ.get("MUJOCO_GL"),
            "error_type": type(exc).__name__,
            "error": type(exc).__name__,
        }
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
