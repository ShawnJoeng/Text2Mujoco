#!/usr/bin/env python3
"""Audit every showcase model for the defects that read as "the robot is broken".

Eight checks run per scene. Each one exists because a real defect shipped past
an earlier version of this guard.

* ``collision_geometry`` - a geom drawn with ``contype="0" conaffinity="0"``
  renders normally and passes through everything. Asking only whether *some*
  geom on a body collides passes a body whose collider is one stub inside a
  large visual shell, so this check also fails an individual non-collidable
  geom that no collider on the same body contains.
* ``declared_mass`` - a geom with neither ``mass`` nor ``density`` compiles at
  density 1000. A gripper palm authored as a 0.14 x 0.075 x 0.05 box then
  weighs 3.5 kg and the servo holding it sags by weight/kp. Writing the mass
  down is the only way the author sees the load they built.
* ``servo_hold`` - command every position servo to hold ``qpos0`` and step for
  two seconds. Nothing that moves may drift. This is the check that catches a
  hand hanging below where it was told to be.
* ``link_continuity`` - two bodies joined by a joint must have geometry that
  meets. A tool on a 0.21 m prismatic with no rendered column is kinematically
  attached and visually detached: the hand looks like it fell off the arm.
  Sampled across the whole replay, because the gap opens mid-stroke.
* ``marker_sites`` - sites never collide, by MuJoCo's design, so nothing the
  robot must avoid can be a site. A marker also has to sit on the surface it
  annotates: a glowing sphere floating in mid-air reads as a bug, and one
  buried inside a bin floor is not a marker at all.
* ``start_pose`` - overlap at ``qpos0`` and after ``reset()``.
* ``sequence_contact`` - the deepest contact of the whole replayed run.
* ``self_overlap`` - a robot needs a collision boundary against itself, not only
  against the scenery. A blanket contype/conaffinity mask that filters
  robot-against-robot turns a finger driven into its own forearm into silence:
  no contact, no complaint. This check demands that every geom pair either be
  tested by MuJoCo, be welded neighbours, or be named in an explicit
  ``<contact><exclude>`` - the one place a model can say "these two are designed
  to nest". Whatever is left is then measured with ``mj_geomDistance``, which
  ignores the filters.

The action tables come from ``capture_sequences.SCENES``, so what is audited is
the schedule the committed captures were produced from. Actions whose id starts
with ``inspect`` are skipped because they only render.

Run with ``MUJOCO_GL=disable``. Exit status is 0 when every scene passes.
"""

from __future__ import annotations

import argparse
import importlib.util
import itertools
import json
import math
import sys
import xml.etree.ElementTree as ElementTree
from pathlib import Path
from typing import Any, Iterable

import mujoco
import numpy as np

SHOWCASE = Path(__file__).resolve().parent
STATIC_LIMIT = -1e-4  # 0.1 mm: the start-pose threshold physics_smoke.py uses
RUN_LIMIT = -1e-3  # 1 mm: below this is solver softness under load, not a gap
SELF_LIMIT = -1e-3  # the same allowance for pairs MuJoCo's filters hide
CONTINUITY_LIMIT = 5e-3  # 5 mm: a seam wider than this is visible in a render
SAG_LIMIT_M = 2e-3  # a servo asked to hold still may drift 2 mm
SAG_LIMIT_RAD = math.radians(1.0)  # and one degree
MARKER_CLEARANCE = 3e-3  # a marker may hover 3 mm over the surface it marks
MARKER_BURIAL = 1e-3  # and may sink 1 mm into it, no more
MARKER_GROUP = 2  # the site group the renderer draws; group 4 holds frames
SAMPLE_STRIDE = 25  # geom-distance sweeps run every 25th step of the replay


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"could not import {path.name}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def geom_name(model: mujoco.MjModel, index: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or f"geom_{index}"


def body_name(model: mujoco.MjModel, index: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index) or f"body_{index}"


def body_geoms(model: mujoco.MjModel, body: int) -> range:
    start = int(model.body_geomadr[body])
    return range(start, start + int(model.body_geomnum[body]))


def moves(model: mujoco.MjModel, body: int) -> bool:
    """True when a body is carried by a joint rather than welded to the world."""
    return int(model.body_weldid[body]) != 0


def free_floating(model: mujoco.MjModel, body: int) -> bool:
    """True when a free joint anywhere above this body makes it a payload.

    A payload is supposed to fall, roll and settle, so the servo-hold check has
    to leave it out or every scene with a ball in it reports a sagging robot.
    """
    while body > 0:
        joints = range(int(model.body_jntadr[body]),
                       int(model.body_jntadr[body]) + int(model.body_jntnum[body]))
        if any(int(model.jnt_type[joint]) == mujoco.mjtJoint.mjJNT_FREE for joint in joints):
            return True
        body = int(model.body_parentid[body])
    return False


def weld_parent(model: mujoco.MjModel, weldid: int) -> int:
    return int(model.body_weldid[int(model.body_parentid[weldid])])


def welded_neighbours(model: mujoco.MjModel, geom_a: int, geom_b: int) -> bool:
    """MuJoCo's structural exclusion: same weld group, or weld parent and child.

    Overlap across such a pair is how a hinge is drawn, not a defect, so both
    the self-overlap sweep and the continuity sweep have to respect it.
    """
    weld_a = int(model.body_weldid[int(model.geom_bodyid[geom_a])])
    weld_b = int(model.body_weldid[int(model.geom_bodyid[geom_b])])
    if weld_a == weld_b:
        return True
    if weld_a != 0 and weld_parent(model, weld_a) == weld_b:
        return True
    return weld_b != 0 and weld_parent(model, weld_b) == weld_a


def filtered_apart(model: mujoco.MjModel, geom_a: int, geom_b: int) -> bool:
    """True when the contype/conaffinity masks keep MuJoCo from testing a pair."""
    type_a, affinity_a = int(model.geom_contype[geom_a]), int(model.geom_conaffinity[geom_a])
    type_b, affinity_b = int(model.geom_contype[geom_b]), int(model.geom_conaffinity[geom_b])
    return not ((type_a & affinity_b) or (type_b & affinity_a))


def declared_exclusion(model: mujoco.MjModel, geom_a: int, geom_b: int) -> bool:
    """True when the model spells out ``<contact><exclude>`` for the two bodies.

    Some overlap is the design: a telescoping ram has to sit inside its sleeve,
    and two links have to interpenetrate at the hinge that joins them. MJCF
    already has a way to say so, and saying so in the model is what separates a
    deliberate nesting from a limb quietly passing through another limb. An
    overlap the author declared is accepted; an overlap hidden behind a
    contype/conaffinity mask is not.
    """
    body_a, body_b = int(model.geom_bodyid[geom_a]), int(model.geom_bodyid[geom_b])
    signatures = {
        (int(signature) >> 16, int(signature) & 0xFFFF)
        for signature in np.asarray(model.exclude_signature).ravel()
    }
    return (body_a, body_b) in signatures or (body_b, body_a) in signatures


def depth_inside(model: mujoco.MjModel, data: mujoco.MjData, geom: int, point: np.ndarray) -> float:
    """How far ``point`` lies inside ``geom``; negative when it is outside.

    ``mj_geomDistance`` compares two geoms, and a site is not a geom, so the
    burial half of the marker check needs its own arithmetic. Only the shapes
    the showcase specs allow are handled.
    """
    local = np.asarray(data.geom_xmat[geom]).reshape(3, 3).T @ (point - np.asarray(data.geom_xpos[geom]))
    size = np.asarray(model.geom_size[geom])
    kind = int(model.geom_type[geom])
    if kind == mujoco.mjtGeom.mjGEOM_PLANE:
        return float(-local[2])
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return float(np.min(size - np.abs(local)))
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return float(size[0] - np.linalg.norm(local))
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return float(min(size[0] - math.hypot(local[0], local[1]), size[1] - abs(local[2])))
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        axial = float(np.clip(local[2], -size[1], size[1]))
        return float(size[0] - np.linalg.norm(local - np.array([0.0, 0.0, axial])))
    if kind == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        radius = float(np.linalg.norm(local / size))
        return float((1.0 - radius) * float(np.min(size)))
    return -1.0  # an unhandled shape is reported as "not buried" rather than guessed


def surface_below(model: mujoco.MjModel, data: mujoco.MjData, point: np.ndarray) -> float:
    """Distance straight down from ``point`` to the first geom, or -1 for none."""
    target = np.zeros(1, dtype=np.int32)
    return float(mujoco.mj_ray(
        model, data, np.asarray(point, dtype=float), np.array([0.0, 0.0, -1.0]),
        None, 1, -1, target,
    ))


def marker_site_names(scene: str) -> list[str]:
    """The sites the scene spec declares as interaction markers.

    Functional sites - a tool centre point, a fork tip - are deliberately in
    mid-air and are not markers, so keying off the spec keeps this check on the
    sites whose whole purpose is to be seen in a render.
    """
    spec = json.loads((SHOWCASE / scene / "scene_spec.json").read_text(encoding="utf-8"))
    names = []
    for point in spec.get("interaction_points", []) or []:
        marker = point.get("marker_site")
        if isinstance(marker, str):
            names.append(marker)
    return names


def site_half_height(model: mujoco.MjModel, site: int) -> float:
    """The site's own downward reach, so "floating" is measured from its surface."""
    size = np.asarray(model.site_size[site])
    kind = int(model.site_type[site])
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return float(size[0])
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return float(size[0] + size[1])
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return float(size[1])
    return float(size[2]) if size[2] > 0 else float(np.max(size))


def collision_geometry_report(model: mujoco.MjModel) -> dict[str, Any]:
    """Per-body and per-geom: everything that moves has to be able to collide."""
    bodies: list[str] = []
    geoms: list[str] = []
    for body in range(1, model.nbody):
        if not moves(model, body):
            continue
        indices = list(body_geoms(model, body))
        colliders = [geom for geom in indices
                     if int(model.geom_contype[geom]) or int(model.geom_conaffinity[geom])]
        if indices and not colliders:
            bodies.append(body_name(model, body))
            continue
        for geom in indices:
            if geom in colliders:
                continue
            centre = np.asarray(model.geom_pos[geom])
            radius = float(model.geom_rbound[geom])
            contained = any(
                float(np.linalg.norm(centre - np.asarray(model.geom_pos[other]))) + radius
                <= float(model.geom_rbound[other]) + 1e-9
                for other in colliders
            )
            if not contained:
                geoms.append(f"{body_name(model, body)}/{geom_name(model, geom)}")
    return {
        "moving_bodies_without_collision": bodies,
        "uncovered_visual_geoms": geoms,
        "passed": not bodies and not geoms,
    }


def declared_mass_report(scene: str) -> dict[str, Any]:
    """Geoms on a moving body that let the compiler pick their mass for them."""
    root = ElementTree.parse(SHOWCASE / scene / "model.xml").getroot()
    undeclared: list[str] = []

    def walk(element: ElementTree.Element, jointed: bool) -> None:
        for body in element.findall("body"):
            carried = jointed or body.find("joint") is not None or body.find("freejoint") is not None
            if carried:
                for geom in body.findall("geom"):
                    if "mass" in geom.attrib or "density" in geom.attrib:
                        continue
                    undeclared.append(f"{body.get('name', '?')}/{geom.get('name', '?')}")
            walk(body, carried)

    world = root.find("worldbody")
    if world is not None:
        walk(world, False)
    return {"geoms_without_declared_mass": undeclared, "passed": not undeclared}


def marker_site_report(scene: str, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
    """Every drawn marker has to lie on the surface it annotates.

    A marker is a ``<site>``, and a site carries no collision geometry at all,
    so it can never stand in for something the robot must not pass through. The
    only honest way to draw one is flush with a solid face: floating in mid-air
    it looks like debris, and sunk inside a bin floor it is not visible at all.

    Group ``2`` is the visible-marker group and every site in it is audited. A
    functional reference site - a tool centre point, a fork tip - belongs in
    group ``4``, which MuJoCo does not draw by default, precisely because it is
    a frame and not a thing.
    """
    floating: list[dict[str, Any]] = []
    buried: list[dict[str, Any]] = []
    grounded: list[str] = []
    colliders = [geom for geom in range(model.ngeom)
                 if int(model.geom_contype[geom]) or int(model.geom_conaffinity[geom])]
    drawn = [site for site in range(model.nsite) if int(model.site_group[site]) == MARKER_GROUP]
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, site) for site in drawn}
    for site in drawn:
        marker = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, site) or f"site_{site}"
        point = np.asarray(data.site_xpos[site], dtype=float)
        reach = site_half_height(model, site)
        deepest = max(
            ((depth_inside(model, data, geom, point), geom) for geom in colliders),
            default=(-1.0, -1),
        )
        if deepest[0] > reach + MARKER_BURIAL:
            buried.append({
                "site": marker,
                "inside": geom_name(model, deepest[1]),
                "depth_mm": round(deepest[0] * 1000.0, 2),
            })
            continue
        if deepest[0] > -MARKER_CLEARANCE:
            grounded.append(marker)  # flush with a face, which is what a decal is
            continue
        drop = surface_below(model, data, point)
        if drop < 0.0:
            floating.append({"site": marker, "detail": "nothing below it at all"})
        elif drop - reach > MARKER_CLEARANCE:
            floating.append({"site": marker, "hover_mm": round((drop - reach) * 1000.0, 2)})
        else:
            grounded.append(marker)
    undrawn = sorted(name for name in marker_site_names(scene) if name not in names)
    return {
        "drawn_markers": len(drawn),
        "grounded": grounded,
        "floating": floating,
        "buried": buried,
        "declared_but_not_drawn": undrawn,
        "passed": not floating and not buried and not undrawn,
    }


def servo_hold_report(model: mujoco.MjModel, seconds: float = 2.0) -> dict[str, Any]:
    """Ask every position servo to hold the start pose and see what gravity does.

    A hand that hangs below its commanded height is the same defect whether the
    cause is an undeclared mass, a kp too small for the load, or a missing
    ``gravcomp`` - and none of the contact checks can see any of them.
    """
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    servos = []
    for actuator in range(model.nu):
        if int(model.actuator_trntype[actuator]) != int(mujoco.mjtTrn.mjTRN_JOINT):
            continue
        if float(model.actuator_gainprm[actuator, 0]) <= 0.0:
            continue
        joint = int(model.actuator_trnid[actuator, 0])
        address = int(model.jnt_qposadr[joint])
        data.ctrl[actuator] = data.qpos[address]
        servos.append((joint, address, int(model.jnt_type[joint])))
    start = data.qpos.copy()
    origin = data.xpos.copy()
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
    sagging = []
    for joint, address, kind in servos:
        error = float(data.qpos[address] - start[address])
        limit = SAG_LIMIT_M if kind == mujoco.mjtJoint.mjJNT_SLIDE else SAG_LIMIT_RAD
        if abs(error) > limit:
            sagging.append({
                "joint": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint),
                "drift_mm" if kind == mujoco.mjtJoint.mjJNT_SLIDE else "drift_deg":
                    round(error * 1000.0, 3) if kind == mujoco.mjtJoint.mjJNT_SLIDE
                    else round(math.degrees(error), 3),
            })
    drift = max(
        (float(np.linalg.norm(np.asarray(data.xpos[body]) - np.asarray(origin[body]))), body)
        for body in range(1, model.nbody)
        if moves(model, body) and not free_floating(model, body)
    ) if any(moves(model, body) and not free_floating(model, body)
             for body in range(1, model.nbody)) else (0.0, 0)
    return {
        "held_s": seconds,
        "sagging_joints": sagging,
        "worst_body": body_name(model, drift[1]),
        "worst_body_drift_mm": round(drift[0] * 1000.0, 3),
        "passed": not sagging and drift[0] <= SAG_LIMIT_M,
    }


def deepest_contact(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, str | None]:
    """The most negative contact gap in the current state, and its pair."""
    worst, pair = 0.0, None
    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.dist < worst:
            worst = float(contact.dist)
            pair = f"{geom_name(model, contact.geom1)} / {geom_name(model, contact.geom2)}"
    return worst, pair


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


def hidden_pairs(model: mujoco.MjModel) -> list[tuple[int, int]]:
    """Geom pairs no contact can ever report, and that the model never owned up to.

    Three things can stop MuJoCo testing a pair: the two geoms are welded
    neighbours, the author wrote ``<contact><exclude>``, or a contype/conaffinity
    mask silently swallowed the pair. The first two are declarations. The third
    is how a robot ends up with no collision boundary against itself, so every
    pair left in this list is a defect on its own, before anything is measured.
    """
    return [
        (geom_a, geom_b)
        for geom_a, geom_b in itertools.combinations(range(model.ngeom), 2)
        if filtered_apart(model, geom_a, geom_b)
        and not welded_neighbours(model, geom_a, geom_b)
        and not declared_exclusion(model, geom_a, geom_b)
    ]


def continuity_pairs(model: mujoco.MjModel) -> list[tuple[str, list[int], list[int]]]:
    """Each jointed body paired with the nearest drawn ancestor it hangs from.

    A free joint is excluded: a payload is not attached to anything, and is not
    supposed to look attached.
    """
    pairs = []
    for body in range(1, model.nbody):
        joints = range(int(model.body_jntadr[body]),
                       int(model.body_jntadr[body]) + int(model.body_jntnum[body]))
        if int(model.body_jntnum[body]) == 0:
            continue
        if any(int(model.jnt_type[joint]) == mujoco.mjtJoint.mjJNT_FREE for joint in joints):
            continue
        child = [geom for geom in body_geoms(model, body)]
        if not child:
            continue
        ancestor = int(model.body_parentid[body])
        while ancestor > 0 and int(model.body_geomnum[ancestor]) == 0:
            ancestor = int(model.body_parentid[ancestor])
        if ancestor == 0:
            continue  # welded straight to the world, which needs no visible seam
        pairs.append((
            f"{body_name(model, ancestor)} -> {body_name(model, body)}",
            list(body_geoms(model, ancestor)),
            child,
        ))
    return pairs


def replay_report(env: Any, config: dict[str, Any]) -> dict[str, Any]:
    """Replay the captured action table, watching contact, self-overlap and seams."""
    model = env.model
    hidden = hidden_pairs(model)
    seams = continuity_pairs(model)
    contact = {"dist": 0.0, "pair": None, "time_s": 0.0}
    overlap: dict[tuple[int, int], tuple[float, float]] = {}
    widest: dict[str, tuple[float, float]] = {}
    steps = 0
    genuine = mujoco.mj_step

    def sweep(data: mujoco.MjData) -> None:
        for geom_a, geom_b in hidden:
            gap = mujoco.mj_geomDistance(model, data, geom_a, geom_b, 0.02, None)
            if gap < SELF_LIMIT and gap < overlap.get((geom_a, geom_b), (0.0, 0.0))[0]:
                overlap[(geom_a, geom_b)] = (gap, float(data.time))
        for label, parent, child in seams:
            gap = min(mujoco.mj_geomDistance(model, data, one, other, 0.5, None)
                      for one in parent for other in child)
            if gap > widest.get(label, (-1.0, 0.0))[0]:
                widest[label] = (gap, float(data.time))

    def watched(watched_model, data, *args, **kwargs):
        nonlocal steps
        genuine(watched_model, data, *args, **kwargs)
        steps += 1
        gap, pair = deepest_contact(watched_model, data)
        if gap < contact["dist"]:
            contact.update(dist=gap, pair=pair, time_s=float(data.time))
        if steps % SAMPLE_STRIDE == 0:
            sweep(data)

    sweep(env.data)
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

    torn = sorted(
        ({"seam": label, "gap_mm": round(gap * 1000.0, 3), "at_s": round(when, 3)}
         for label, (gap, when) in widest.items() if gap > CONTINUITY_LIMIT),
        key=lambda item: -item["gap_mm"],
    )
    penetrating = sorted(
        ({"pair": f"{geom_name(model, one)} / {geom_name(model, other)}",
          "overlap_mm": round(-gap * 1000.0, 3), "at_s": round(when, 3)}
         for (one, other), (gap, when) in overlap.items()),
        key=lambda item: -item["overlap_mm"],
    )
    return {
        "sequence_contact": {
            "steps": steps,
            "simulated_s": round(float(env.data.time), 3),
            "deepest_overlap_mm": round(-contact["dist"] * 1000.0, 4),
            "deepest_pair": contact["pair"] if contact["dist"] < RUN_LIMIT else None,
            "deepest_at_s": round(contact["time_s"], 3) if contact["dist"] < RUN_LIMIT else None,
            "passed": contact["dist"] >= RUN_LIMIT,
        },
        "self_overlap": {
            "undeclared_filtered_pairs": len(hidden),
            "sample": [f"{geom_name(model, one)} / {geom_name(model, other)}"
                       for one, other in hidden[:12]],
            "penetrating": penetrating,
            "passed": not hidden and not penetrating,
        },
        "link_continuity": {
            "seams_checked": len(seams),
            "widest_seam_mm": round(max((gap for gap, _ in widest.values()), default=0.0) * 1000.0, 3),
            "torn": torn,
            "passed": not torn,
        },
    }


def build(scene: str) -> Any:
    package = SHOWCASE / scene
    module = load_module(package / "environment.py", f"showcase_env_{scene}")
    factory = next(
        (getattr(module, name) for name in
         ("build_environment", "make_environment", "create_environment")
         if hasattr(module, name)),
        None,
    )
    if factory is None:
        raise SystemExit(f"{scene}/environment.py exposes no environment factory")
    return factory(package / "model.xml", package / "scene_spec.json")


def audit(scene: str, config: dict[str, Any]) -> dict[str, Any]:
    env = build(scene)
    fresh = mujoco.MjData(env.model)
    mujoco.mj_forward(env.model, fresh)
    result: dict[str, Any] = {
        "scene": scene,
        "collision_geometry": collision_geometry_report(env.model),
        "declared_mass": declared_mass_report(scene),
        "marker_sites": marker_site_report(scene, env.model, fresh),
        "servo_hold": servo_hold_report(env.model),
        "start_pose": start_pose_report(env),
    }
    result.update(replay_report(env, config))
    result["passed"] = all(
        result[check]["passed"] for check in
        ("collision_geometry", "declared_mass", "marker_sites", "servo_hold",
         "start_pose", "sequence_contact", "self_overlap", "link_continuity")
    )
    return result


def describe(result: dict[str, Any]) -> None:
    print(f"== {result['scene']}   {'PASS' if result['passed'] else 'FAIL'}")

    collision = result["collision_geometry"]
    if collision["passed"]:
        print("   collision geometry  every moving geom collides")
    else:
        for body in collision["moving_bodies_without_collision"]:
            print(f"   collision geometry  NO COLLIDER on moving body {body}")
        for geom in collision["uncovered_visual_geoms"]:
            print(f"   collision geometry  visual-only geom outside the envelope: {geom}")

    mass = result["declared_mass"]
    undeclared = mass["geoms_without_declared_mass"]
    print(f"   declared mass       {'every moving geom states its mass' if mass['passed'] else str(len(undeclared)) + ' geoms compiled at density 1000: ' + ', '.join(undeclared[:4])}")

    markers = result["marker_sites"]
    if markers["passed"]:
        print(f"   marker sites        {markers['drawn_markers']} drawn markers, all flush with a surface")
    else:
        for item in markers["floating"]:
            print(f"   marker sites        FLOATING {item['site']} {item.get('hover_mm', item.get('detail'))}")
        for item in markers["buried"]:
            print(f"   marker sites        BURIED {item['site']} {item['depth_mm']} mm inside {item['inside']}")
        for undrawn in markers["declared_but_not_drawn"]:
            print(f"   marker sites        NOT DRAWN {undrawn} is declared but is not a group-2 site")

    hold = result["servo_hold"]
    if hold["passed"]:
        print(f"   servo hold          holds qpos0, worst drift {hold['worst_body_drift_mm']:.3f} mm")
    else:
        print(f"   servo hold          SAG {hold['worst_body']} drifts {hold['worst_body_drift_mm']:.3f} mm"
              + (f" via {', '.join(str(item) for item in hold['sagging_joints'])}" if hold["sagging_joints"] else ""))

    start = result["start_pose"]
    print(f"   start pose          qpos0 {start['qpos0_overlap_mm']:8.4f} mm"
          f"   post-reset {start['post_reset_overlap_mm']:8.4f} mm"
          f"   {'clean' if start['passed'] else 'PENETRATION'}")

    run = result["sequence_contact"]
    detail = f"  <- {run['deepest_pair']} at t={run['deepest_at_s']} s" if run["deepest_pair"] else ""
    print(f"   sequence contact    {run['steps']} steps to {run['simulated_s']} s,"
          f" deepest {run['deepest_overlap_mm']:8.4f} mm"
          f"   {'within solver softness' if run['passed'] else 'PENETRATION'}{detail}")

    self_overlap = result["self_overlap"]
    if self_overlap["passed"]:
        print("   self overlap        every pair either collides or is a declared <exclude>")
    else:
        if self_overlap["undeclared_filtered_pairs"]:
            print(f"   self overlap        {self_overlap['undeclared_filtered_pairs']} pairs hidden by"
                  f" collision masks, e.g. {', '.join(self_overlap['sample'][:2])}")
        for item in self_overlap["penetrating"][:4]:
            print(f"   self overlap        {item['overlap_mm']} mm at t={item['at_s']} s   {item['pair']}")

    seams = result["link_continuity"]
    if seams["passed"]:
        print(f"   link continuity     {seams['seams_checked']} seams, widest {seams['widest_seam_mm']:.3f} mm")
    else:
        for item in seams["torn"][:4]:
            print(f"   link continuity     TORN {item['seam']}: {item['gap_mm']} mm at t={item['at_s']} s")


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
    print(f"\nscenes passing the eight-check model audit: {passed}/{len(results)}")
    if args.report:
        destination = (SHOWCASE / args.report).resolve() if not args.report.is_absolute() else args.report.resolve()
        if SHOWCASE.resolve() not in destination.parents:
            raise SystemExit("report must be written inside the showcase directory")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps({
                "static_limit_mm": 0.1,
                "run_limit_mm": 1.0,
                "self_overlap_limit_mm": 1.0,
                "seam_limit_mm": CONTINUITY_LIMIT * 1000.0,
                "sag_limit_mm": SAG_LIMIT_M * 1000.0,
                "marker_clearance_mm": MARKER_CLEARANCE * 1000.0,
                "scenes": results,
            }, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"report written to {destination.relative_to(SHOWCASE)}")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
