#!/usr/bin/env python3
"""Validate the canonical interaction manifest for every showcase scene."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
POINT_FIELDS = (
    "target",
    "marker_site",
    "pose",
    "affordance",
    "preconditions",
    "success_conditions",
    "depends_on",
    "effects",
    "reset",
    "action",
)


def load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AssertionError(f"{path} must contain a JSON object")
    return value


def validate_scene(scene_dir: Path) -> None:
    spec = load(scene_dir / "scene_spec.json")
    manifest = load(scene_dir / "interaction_manifest.json")
    for field in ("schema_version", "backend", "source", "scene", "action_envelope", "interaction_points", "dependency_order"):
        if field not in manifest:
            raise AssertionError(f"{scene_dir.name}: manifest missing {field}")
    if manifest["schema_version"] != spec.get("schema_version"):
        raise AssertionError(f"{scene_dir.name}: schema version mismatch")
    if manifest["backend"] != spec.get("backend") or manifest["source"] != "scene_spec.json":
        raise AssertionError(f"{scene_dir.name}: manifest metadata mismatch")
    envelope = manifest["action_envelope"]
    if not isinstance(envelope, dict) or envelope.get("required") != ["id", "payload"]:
        raise AssertionError(f"{scene_dir.name}: invalid action envelope")

    if not isinstance(manifest["interaction_points"], list):
        raise AssertionError(f"{scene_dir.name}: manifest interaction_points must be a list")
    spec_list = spec.get("interaction_points", [])
    if not isinstance(spec_list, list):
        raise AssertionError(f"{scene_dir.name}: spec interaction_points must be a list")
    spec_points = {point["id"]: point for point in spec_list}
    manifest_points = {point["id"]: point for point in manifest["interaction_points"]}
    if len(spec_points) != len(spec_list) or len(manifest_points) != len(manifest["interaction_points"]):
        raise AssertionError(f"{scene_dir.name}: duplicate interaction IDs")
    if set(spec_points) != set(manifest_points):
        raise AssertionError(f"{scene_dir.name}: interaction IDs differ")
    if manifest["dependency_order"] != [point["id"] for point in spec_list]:
        raise AssertionError(f"{scene_dir.name}: dependency order differs")
    for point_id, spec_point in spec_points.items():
        manifest_point = manifest_points[point_id]
        for field in POINT_FIELDS:
            if manifest_point.get(field) != spec_point.get(field):
                raise AssertionError(f"{scene_dir.name}/{point_id}: {field} differs")


def main() -> int:
    for scene_dir in sorted(ROOT.glob("0*")):
        if scene_dir.is_dir():
            validate_scene(scene_dir)
            print(f"{scene_dir.name}: PASS")
    print("manifest validation: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
