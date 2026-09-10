#!/usr/bin/env python3
"""Static-contract and MuJoCo physics smoke test."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from environment import EnvironmentError, build_environment, xyzw_to_wxyz


def expect_error(call, label: str) -> None:
    try:
        call()
    except EnvironmentError:
        return
    raise AssertionError(f"expected EnvironmentError for {label}")


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


def run(args: argparse.Namespace) -> dict:
    if os.environ.get("MUJOCO_GL") != "disable":
        raise AssertionError("physics smoke must run with MUJOCO_GL=disable")
    ET.parse(args.model)
    environment = build_environment(args.model, args.spec)
    initial_contact_audit(environment)
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

    with SequenceContactAudit(environment.model) as sequence_audit:
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

        # The documented lower boundary must be executable, not rejected by the
        # actuator convergence tolerance.
        environment.reset()
        boundary = environment.step(
            {"id": "press_unlock_button", "payload": {"press_depth_m": 0.01}}
        )
        if boundary["unlock_button_joint_qpos"] < 0.01 - 1e-5:
            raise AssertionError("minimum legal button depth did not unlock")
        environment.reset()

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
    sequence_contact = sequence_audit.report()

    artifacts = environment.save_artifacts(args.output_dir)
    xml_reload = mujoco.MjModel.from_xml_path(
        str(environment.package_root / artifacts["mjcf"])
    )
    binary_reload = mujoco.MjModel.from_binary_path(
        str(environment.package_root / artifacts["mjb"])
    )
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
        "path_base": "package_root",
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "mjcf_compile": "PASS",
        "initial_contact": "PASS",
        "sequence_contact": sequence_contact,
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
    parser.add_argument("--result", type=Path, default=base / "output" / "physics_results.json")
    args = parser.parse_args()
    args.model = args.model.resolve()
    args.spec = args.spec.resolve()
    args.output_dir = args.output_dir.resolve()
    args.result = args.result.resolve()
    try:
        result, exit_code = run(args), 0
    except Exception as exc:
        result, exit_code = {
            "status": "FAIL",
            "mujoco_executed": True,
            "mujoco_version": mujoco.__version__,
            "python_version": platform.python_version(),
            "path_base": "package_root",
            "mujoco_gl": os.environ.get("MUJOCO_GL"),
            "error_type": type(exc).__name__,
            "error": type(exc).__name__,
        }, 1
    args.result.parent.mkdir(parents=True, exist_ok=True)
    args.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
