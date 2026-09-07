#!/usr/bin/env python3
"""Static contract checks for the generated Text2MuJoCo package."""

from __future__ import annotations

import json
import py_compile
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any


def _floats(value: str) -> list[float]:
    return [float(item) for item in value.split()]


def _assert_close(actual: list[float], expected: list[float], label: str) -> None:
    if len(actual) != len(expected) or any(
        abs(left - right) > 1e-9 for left, right in zip(actual, expected)
    ):
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def _xml_names(root: ET.Element) -> dict[str, set[str]]:
    names: dict[str, set[str]] = defaultdict(set)
    for tag, object_type in (
        ("body", "body"),
        ("geom", "geom"),
        ("joint", "joint"),
        ("freejoint", "joint"),
        ("site", "site"),
        ("camera", "camera"),
    ):
        for element in root.iter(tag):
            if "name" in element.attrib:
                name = element.attrib["name"]
                if name in names[object_type]:
                    raise AssertionError(f"duplicate XML {object_type} name: {name}")
                names[object_type].add(name)
    actuator = root.find("actuator")
    if actuator is not None:
        for element in actuator:
            if "name" in element.attrib:
                name = element.attrib["name"]
                if name in names["actuator"]:
                    raise AssertionError(f"duplicate XML actuator name: {name}")
                names["actuator"].add(name)
    return names


def main() -> int:
    base = Path(__file__).resolve().parent
    spec: dict[str, Any] = json.loads((base / "scene_spec.json").read_text())
    manifest: dict[str, Any] = json.loads(
        (base / "interaction_manifest.json").read_text()
    )
    root = ET.parse(base / "model.xml").getroot()
    if root.tag != "mujoco":
        raise AssertionError("model.xml root must be <mujoco>")

    spec_points = {point["id"]: point for point in spec["interaction_points"]}
    manifest_points = {point["id"]: point for point in manifest["points"]}
    if set(spec_points) != set(manifest_points):
        raise AssertionError("manifest interaction IDs differ from scene spec")
    required = {
        "target",
        "pose",
        "affordance",
        "activation",
        "action_schema",
        "preconditions",
        "success_conditions",
        "depends_on",
        "effects",
        "reset",
        "status",
    }
    for point_id, manifest_point in manifest_points.items():
        missing = required - manifest_point.keys()
        if missing:
            raise AssertionError(f"{point_id} manifest fields missing: {sorted(missing)}")
        source = spec_points[point_id]
        comparisons = {
            "target": source["target"],
            "pose": source["pose"],
            "affordance": source["affordance"],
            "action_schema": source["action"]["schema"],
            "preconditions": source["preconditions"],
            "success_conditions": source["success_conditions"],
            "depends_on": source["depends_on"],
            "effects": source["effects"],
            "reset": source["reset"],
        }
        for field, expected in comparisons.items():
            if manifest_point[field] != expected:
                raise AssertionError(f"{point_id} {field} mismatch")
        if "marker_site" in source:
            if manifest_point.get("marker_site") != source["marker_site"]:
                raise AssertionError(f"{point_id} marker_site mismatch")
        elif "marker_site" in manifest_point:
            raise AssertionError(f"{point_id} unexpectedly declares marker_site")
        if manifest_point["activation"] != {
            "mode": source["action"]["mode"],
            "command": source["action"]["command"],
        }:
            raise AssertionError(f"{point_id} activation mismatch")

    names = _xml_names(root)
    for point in spec["interaction_points"]:
        if point.get("marker_site") and point["marker_site"] not in names["site"]:
            raise AssertionError(f"missing marker site: {point['marker_site']}")
        target = point["target"]
        if target["name"] not in names[target["type"]]:
            raise AssertionError(f"missing typed target: {target}")

    geoms = {element.attrib["name"]: element for element in root.iter("geom") if "name" in element.attrib}
    _assert_close(_floats(geoms["table_geom"].attrib["size"]), [0.6, 0.4, 0.025], "table half-size")
    _assert_close(_floats(geoms["red_cube_geom"].attrib["size"]), [0.05, 0.05, 0.05], "cube half-size")
    _assert_close(_floats(geoms["start_button_geom"].attrib["size"]), [0.04, 0.015], "cylinder radius/half-height")
    for name in ("box_bottom", "box_left", "box_right", "box_front", "box_back"):
        if name not in geoms or geoms[name].attrib.get("type") != "box":
            raise AssertionError(f"open-box collider missing: {name}")

    for filename in (
        "environment.py",
        "contract_test.py",
        "validator_unit_test.py",
        "runtime_probe.py",
        "physics_smoke.py",
        "render_smoke.py",
    ):
        py_compile.compile(str(base / filename), doraise=True)

    result = {
        "status": "PASS",
        "json_parse": "PASS",
        "xml_parse": "PASS",
        "manifest_consistency": "PASS",
        "typed_target_resolution": "PASS",
        "geometry_conversion": "PASS",
        "python_compile": "PASS",
        "interaction_ids": list(spec_points),
    }
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise
