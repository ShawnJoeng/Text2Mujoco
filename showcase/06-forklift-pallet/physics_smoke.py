#!/usr/bin/env python3
"""Physics and interaction smoke test for the forklift delivery cell."""

from __future__ import annotations

import argparse
import json
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
    for object_type, names in {
        "body": ["forklift_base", "forklift_mast", "fork_carriage", "left_fork", "right_fork", "pallet", "payload_crate", "delivery_zone"],
        "joint": ["forklift_x", "forklift_y", "forklift_yaw", "fork_lift", "pallet_free"],
        "actuator": ["forklift_x_motor", "forklift_y_motor", "forklift_yaw_motor", "fork_lift_motor"],
        "site": [point["marker_site"] for point in env.spec["interaction_points"]] + ["fork_tip_site"],
    }.items():
        for name in names:
            env.require_id(object_type, name)
    if env.model.nu != 4 or env.model.njnt != 5:
        raise AssertionError("forklift joint/actuator count is unexpected")
    if xyzw_to_wxyz([0, 0, 0, 1]) != [1.0, 0.0, 0.0, 0.0]:
        raise AssertionError("quaternion conversion failed")
    expected = [
        "drive_to_pallet",
        "raise_forks",
        "engage_pallet",
        "carry_to_drop_zone",
        "lower_forks_release",
        "inspect_forklift_delivery",
    ]
    if [point["id"] for point in env.list_interaction_points()] != expected:
        raise AssertionError("unexpected interaction ordering")
    return {
        "xml_parse": "PASS",
        "mjcf_compile": "PASS",
        "mobile_joints": ["forklift_x", "forklift_y", "forklift_yaw"],
        "fork_lift_joint": "fork_lift",
        "actuators": 4,
        "typed_targets": "PASS",
        "visible_markers": "PASS",
        "quaternion_conversion": "PASS",
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
    env = build_environment(args.model, args.spec)
    initial_contact_audit(env)
    checks = static_checks(env)
    initial = env.observe()
    if initial["state"]["forklift_state"] != "parked" or initial["state"]["pallet_state"] != "loaded":
        raise AssertionError("reset did not initialize forklift state")
    expect_error(lambda: env.step({"id": "raise_forks", "payload": {"lift_m": 0.18}}), "lift before alignment")
    expect_error(lambda: env.step({"id": "drive_to_pallet", "payload": {"speed_mps": 0.1}}), "speed lower bound")
    expect_error(lambda: env.step({"id": "drive_to_pallet", "payload": {"unknown": 1}}), "unknown payload")
    expect_error(lambda: env.step({"id": "missing", "payload": {}}), "unknown interaction")
    expect_error(lambda: env.reset(seed=True), "boolean seed")
    env.run_physics(40)
    if not np.all(np.isfinite(env.data.qpos)) or not np.all(np.isfinite(env.data.qvel)):
        raise AssertionError("free physics produced a non-finite state")

    with SequenceContactAudit(env.model) as audit:
        aligned = env.step({"id": "drive_to_pallet", "payload": {"speed_mps": 0.8}})
        if aligned["state"]["forklift_state"] != "aligned":
            raise AssertionError("forklift did not align with pallet")
        if float(np.linalg.norm(np.asarray(aligned["fork_anchor_position"])[:2] - np.asarray(aligned["pallet_position"])[:2])) > 0.14:
            raise AssertionError("fork anchor is too far from pallet")

        raised = env.step({"id": "raise_forks", "payload": {"lift_m": 0.18}})
        if raised["fork_lift_qpos_m"] < 0.15 or raised["state"]["forklift_state"] != "raised":
            raise AssertionError("forks did not raise")
        expect_error(lambda: env.step({"id": "engage_pallet", "payload": {"confirm": False}}), "engage flag")
        engaged = env.step({"id": "engage_pallet", "payload": {"confirm": True}})
        if engaged["state"]["pallet_state"] != "engaged":
            raise AssertionError("pallet was not engaged")
        pallet_before = np.asarray(engaged["pallet_position"], dtype=float)
        carried = env.step({"id": "carry_to_drop_zone", "payload": {"speed_mps": 0.8}})
        pallet_after = np.asarray(carried["pallet_position"], dtype=float)
        if np.linalg.norm(pallet_after[:2] - pallet_before[:2]) < 0.8:
            raise AssertionError("engaged pallet did not follow forklift motion")
        if carried["rack_collision_count"] != 0 or carried["state"]["forklift_state"] != "at_delivery":
            raise AssertionError("delivery route contacted the storage rack")
        expect_error(lambda: env.step({"id": "lower_forks_release", "payload": {"release": False}}), "release flag")
        released = env.step({"id": "lower_forks_release", "payload": {"release": True}})
        if released["state"]["pallet_state"] != "delivered" or not released["pallet_inside_delivery"]:
            raise AssertionError("pallet did not settle inside delivery zone")
        if released["fork_lift_qpos_m"] > 0.05:
            raise AssertionError("forks did not lower")
        if env.is_success():
            raise AssertionError("inspection must be required for success")
    sequence_contact = audit.report()

    artifact_root = env.package_root / "output"
    artifact_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="text2mujoco-forklift-", dir=artifact_root) as temp_dir:
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
    if reset["history"] or reset["state"]["pallet_state"] != "loaded" or reset["fork_lift_qpos_m"] != 0.0:
        raise AssertionError("reset did not restore initial state")
    return {
        "status": "PASS",
        "mujoco_executed": True,
        "mujoco_version": mujoco.__version__,
        "python_version": platform.python_version(),
        "path_base": "package_root",
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "static_checks": checks,
        "physics": {"initial_contact": "PASS", "mj_step": "PASS", "finite_state": "PASS"},
        "interaction_sequence": ["drive_to_pallet", "raise_forks", "engage_pallet", "carry_to_drop_zone", "lower_forks_release"],
        "pallet_sync": "PASS",
        "sequence_contact": sequence_contact,
        "route_collision_count": env.rack_collision_count,
        "delivery_zone": "PASS",
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
