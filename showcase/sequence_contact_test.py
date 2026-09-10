#!/usr/bin/env python3
"""Audit collision geometry and sequence-wide contact for every showcase scene.

Two failures are easy to ship and hard to see, so this test looks for both.

* A body drawn with ``contype="0" conaffinity="0"`` renders normally, carries
  mass, and passes through every other geom without generating a contact. An
  arm built that way sweeps through its own table while all task assertions
  pass, which is why the static half of this test rejects any moving body with
  no collidable geom.
* A ``t=0`` contact audit describes the start pose only. Endpoint tolerances
  describe the ends. Neither sees the middle of a transfer, so the dynamic half
  replays each scene's documented action table with ``mujoco.mj_step`` wrapped
  and keeps the deepest contact of the whole run.

The action tables come from ``capture_sequences.SCENES``, so what is audited is
the same schedule the committed captures were produced from. Actions whose id
starts with ``inspect`` are skipped because they only render; each scene's own
``physics_smoke.py`` covers the full sequence including inspection.

Run with ``MUJOCO_GL=disable``. Exit status is 0 when every scene passes.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, Iterable

import mujoco

SHOWCASE = Path(__file__).resolve().parent
STATIC_LIMIT = -1e-4  # 0.1 mm: the start-pose threshold physics_smoke.py uses
RUN_LIMIT = -1e-3  # 1 mm: below this is solver softness under load, not a gap


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"could not import {path.name}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def deepest_contact(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, str | None]:
    """Return the most negative contact gap in the current state and its pair."""
    worst, pair = 0.0, None
    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.dist < worst:
            worst = float(contact.dist)
            pair = "{} / {}".format(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
            )
    return worst, pair


def uncollidable_moving_bodies(model: mujoco.MjModel) -> list[str]:
    """Moving bodies whose every geom is excluded from collision.

    ``body_weldid`` is the root of the rigid weld group, so a body welded to the
    world has ``body_weldid == 0``. Anything else is carried by a joint, either
    its own or an ancestor's, and therefore has to collide.
    """
    offenders = []
    for body in range(1, model.nbody):
        if int(model.body_weldid[body]) == 0:
            continue
        start = int(model.body_geomadr[body])
        count = int(model.body_geomnum[body])
        geoms = range(start, start + count)
        if not any(int(model.geom_contype[geom]) or int(model.geom_conaffinity[geom])
                   for geom in geoms):
            offenders.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or f"body_{body}")
    return offenders


def build(scene: str) -> Any:
    package = SHOWCASE / scene
    module = load_module(package / "environment.py", f"showcase_env_{scene}")
    factory = next(
        (getattr(module, name) for name in ("build_environment", "make_environment", "create_environment")
         if hasattr(module, name)),
        None,
    )
    if factory is None:
        raise SystemExit(f"{scene}/environment.py exposes no environment factory")
    return factory(package / "model.xml", package / "scene_spec.json")


def start_pose_report(env: Any) -> dict[str, Any]:
    fresh = mujoco.MjData(env.model)
    mujoco.mj_forward(env.model, fresh)
    compiled = deepest_contact(env.model, fresh)
    env.reset(seed=0)
    mujoco.mj_forward(env.model, env.data)
    after_reset = deepest_contact(env.model, env.data)
    return {
        "qpos0_overlap_mm": round(-compiled[0] * 1000.0, 4),
        "qpos0_pair": compiled[1] if compiled[0] < STATIC_LIMIT else None,
        "post_reset_overlap_mm": round(-after_reset[0] * 1000.0, 4),
        "post_reset_pair": after_reset[1] if after_reset[0] < STATIC_LIMIT else None,
        "passed": compiled[0] >= STATIC_LIMIT and after_reset[0] >= STATIC_LIMIT,
    }


def replay_report(env: Any, config: dict[str, Any]) -> dict[str, Any]:
    """Replay the captured action table, watching every physics step."""
    worst = {"dist": 0.0, "pair": None, "time_s": 0.0}
    steps = 0
    genuine = mujoco.mj_step

    def watched(model, data, *args, **kwargs):
        nonlocal steps
        genuine(model, data, *args, **kwargs)
        steps += 1
        dist, pair = deepest_contact(model, data)
        if dist < worst["dist"]:
            worst.update(dist=dist, pair=pair, time_s=float(data.time))

    mujoco.mj_step = watched
    try:
        if config.get("warmup_steps"):
            env.run_physics(int(config["warmup_steps"]))
        for identifier, payload in config["actions"]:
            if identifier.startswith("inspect"):
                continue
            env.step({"id": identifier, "payload": dict(payload)})
            settle = (config.get("settle_after") or {}).get(identifier)
            if settle:
                env.run_physics(int(settle))
    finally:
        mujoco.mj_step = genuine
    return {
        "steps": steps,
        "simulated_s": round(float(env.data.time), 3),
        "deepest_overlap_mm": round(-worst["dist"] * 1000.0, 4),
        "deepest_pair": worst["pair"] if worst["dist"] < RUN_LIMIT else None,
        "deepest_at_s": round(worst["time_s"], 3) if worst["dist"] < RUN_LIMIT else None,
        "passed": worst["dist"] >= RUN_LIMIT,
    }


def audit(scene: str, config: dict[str, Any]) -> dict[str, Any]:
    env = build(scene)
    visual_only = uncollidable_moving_bodies(env.model)
    result: dict[str, Any] = {
        "scene": scene,
        "collision_geometry": {
            "moving_bodies_without_collision": visual_only,
            "passed": not visual_only,
        },
        "start_pose": start_pose_report(env),
    }
    result["sequence"] = replay_report(env, config)
    result["passed"] = all(
        section["passed"] for section in
        (result["collision_geometry"], result["start_pose"], result["sequence"])
    )
    return result


def describe(result: dict[str, Any]) -> None:
    collision = result["collision_geometry"]
    start = result["start_pose"]
    run = result["sequence"]
    print(f"== {result['scene']}   {'PASS' if result['passed'] else 'FAIL'}")
    if collision["passed"]:
        print("   collision geometry  every moving body collides")
    else:
        print("   collision geometry  NO COLLIDER: "
              + ", ".join(collision["moving_bodies_without_collision"]))
    print(f"   start pose          qpos0 {start['qpos0_overlap_mm']:8.4f} mm"
          f"   post-reset {start['post_reset_overlap_mm']:8.4f} mm"
          f"   {'clean' if start['passed'] else 'PENETRATION'}")
    detail = f"  <- {run['deepest_pair']} at t={run['deepest_at_s']} s" if run["deepest_pair"] else ""
    print(f"   sequence            {run['steps']} steps to {run['simulated_s']} s,"
          f" deepest {run['deepest_overlap_mm']:8.4f} mm"
          f"   {'within solver softness' if run['passed'] else 'PENETRATION'}{detail}")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenes", nargs="*", help="scene directory names; default is every captured scene")
    parser.add_argument("--report", type=Path, help="write the JSON report to this package-relative path")
    args = parser.parse_args(list(argv) if argv is not None else None)

    capture = load_module(SHOWCASE / "capture_sequences.py", "showcase_capture_sequences")
    scenes = args.scenes or list(capture.SCENES)
    unknown = [scene for scene in scenes if scene not in capture.SCENES]
    if unknown:
        raise SystemExit(f"no captured action table for: {', '.join(unknown)}")

    results = []
    for scene in scenes:
        result = audit(scene, capture.SCENES[scene])
        describe(result)
        results.append(result)

    passed = sum(1 for result in results if result["passed"])
    print(f"\nscenes passing collision and sequence contact audit: {passed}/{len(results)}")
    if args.report:
        destination = (SHOWCASE / args.report).resolve() if not args.report.is_absolute() else args.report.resolve()
        if SHOWCASE.resolve() not in destination.parents:
            raise SystemExit("report must be written inside the showcase directory")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps({"static_limit_mm": 0.1, "run_limit_mm": 1.0, "scenes": results}, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"report written to {destination.relative_to(SHOWCASE)}")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
