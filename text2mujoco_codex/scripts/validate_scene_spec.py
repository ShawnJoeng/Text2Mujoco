#!/usr/bin/env python3
"""Validate the portable scene contract used by the text2mujoco skill."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


ALLOWED_ACTION_MODES = {"direct", "keyboard", "viewer", "ros2"}
ALLOWED_ASSET_KINDS = {"primitive", "mesh", "mjcf_include", "robot"}
ALLOWED_GL_BACKENDS = {"auto", "disable", "egl", "osmesa", "glfw", "cgl"}
ALLOWED_SHAPES = {"box", "cylinder", "sphere", "capsule", "plane", "open_box"}
ALLOWED_TARGET_TYPES = {"body", "geom", "joint", "actuator", "site", "camera"}
ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
NAME_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*$")


class Reporter:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


def is_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def is_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def check_vector(value: Any, length: int, label: str, report: Reporter) -> bool:
    if (
        not isinstance(value, list)
        or len(value) != length
        or not all(is_number(item) for item in value)
    ):
        report.error(f"{label} must be a list of {length} finite numbers")
        return False
    return True


def check_string_list(value: Any, label: str, report: Reporter) -> bool:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        report.error(f"{label} must be a list of strings")
        return False
    return True


def check_condition_list(value: Any, label: str, report: Reporter) -> bool:
    if not isinstance(value, list):
        report.error(f"{label} must be a list")
        return False
    valid = True
    for index, condition in enumerate(value):
        if isinstance(condition, str):
            if not condition.strip():
                report.error(f"{label}[{index}] must not be empty")
                valid = False
        elif isinstance(condition, dict):
            if not isinstance(condition.get("op"), str) or not condition["op"].strip():
                report.error(f"{label}[{index}].op must be a non-empty string")
                valid = False
            if not isinstance(condition.get("path"), str) or not condition["path"].strip():
                report.error(f"{label}[{index}].path must be a non-empty string")
                valid = False
        else:
            report.error(f"{label}[{index}] must be a string or object")
            valid = False
    return valid


def check_name(value: Any, label: str, report: Reporter) -> bool:
    if not isinstance(value, str) or not NAME_PATTERN.fullmatch(value):
        report.error(f"{label} must be a stable MuJoCo name matching {NAME_PATTERN.pattern}")
        return False
    return True


def check_pose(value: Any, label: str, report: Reporter) -> None:
    if not isinstance(value, dict):
        report.error(f"{label} must be an object")
        return
    check_vector(value.get("position"), 3, f"{label}.position", report)
    quaternion = value.get("orientation_xyzw")
    if quaternion is not None and check_vector(
        quaternion, 4, f"{label}.orientation_xyzw", report
    ):
        norm = math.sqrt(sum(component * component for component in quaternion))
        if norm < 1e-8:
            report.error(f"{label}.orientation_xyzw must not be zero")
        elif abs(norm - 1.0) > 1e-2:
            report.warning(
                f"{label}.orientation_xyzw is not normalized (norm={norm:.4f})"
            )


def check_unique_ids(items: Any, label: str, report: Reporter) -> tuple[list[Any], set[str]]:
    if not isinstance(items, list):
        report.error(f"{label} must be a list")
        return [], set()
    seen: set[str] = set()
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            report.error(f"{label}[{index}] must be an object")
            continue
        item_id = item.get("id")
        if not isinstance(item_id, str) or not ID_PATTERN.fullmatch(item_id):
            report.error(f"{label}[{index}].id must match {ID_PATTERN.pattern}")
            continue
        if item_id in seen:
            report.error(f"duplicate {label} id: {item_id}")
        seen.add(item_id)
    return items, seen


def add_object_name(
    namespaces: dict[str, set[str]],
    object_type: str,
    value: Any,
    label: str,
    report: Reporter,
) -> None:
    if not check_name(value, label, report):
        return
    if value in namespaces[object_type]:
        report.error(f"duplicate MuJoCo {object_type} name: {value}")
    namespaces[object_type].add(value)


def check_dimensions(
    dimensions: Any, shape: Any, label: str, report: Reporter
) -> None:
    if not check_vector(dimensions, 3, label, report):
        return
    if any(value <= 0 for value in dimensions):
        report.error(f"{label} values must be positive full dimensions")
        return
    if shape == "cylinder" and not math.isclose(dimensions[0], dimensions[1]):
        report.error(f"{label} cylinder x/y diameters must match")
    if shape == "capsule" and not math.isclose(dimensions[0], dimensions[1]):
        report.error(f"{label} capsule x/y diameters must match")
    if shape == "sphere" and not (
        math.isclose(dimensions[0], dimensions[1])
        and math.isclose(dimensions[1], dimensions[2])
    ):
        report.error(f"{label} sphere diameters must match")


def check_bounds(value: Any, label: str, report: Reporter) -> None:
    if not isinstance(value, dict):
        report.error(f"{label} must be an object")
        return
    minimum = value.get("min")
    maximum = value.get("max")
    if check_vector(minimum, 3, f"{label}.min", report) and check_vector(
        maximum, 3, f"{label}.max", report
    ):
        if any(lower >= upper for lower, upper in zip(minimum, maximum)):
            report.error(f"{label}.min must be less than max")


def check_dependencies(points: list[Any], point_ids: set[str], report: Reporter) -> None:
    graph: dict[str, list[str]] = {}
    point_indices = {
        point.get("id"): index
        for index, point in enumerate(points)
        if isinstance(point, dict) and isinstance(point.get("id"), str)
    }
    for index, point in enumerate(points):
        if not isinstance(point, dict) or not isinstance(point.get("id"), str):
            continue
        dependencies = point.get("depends_on")
        if not isinstance(dependencies, list):
            report.error(f"interaction_points[{index}].depends_on must be a list")
            continue
        graph[point["id"]] = []
        for dependency in dependencies:
            if not isinstance(dependency, str) or dependency not in point_ids:
                report.error(
                    f"interaction_points[{index}].depends_on references unknown id: {dependency}"
                )
            elif dependency == point["id"]:
                report.error(f"interaction point cannot depend on itself: {dependency}")
            elif point_indices.get(dependency, index) >= index:
                report.error(
                    f"interaction_points[{index}].depends_on must reference an earlier point: {dependency}"
                )
            else:
                graph[point["id"]].append(dependency)

    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(node: str) -> None:
        if node in visiting:
            report.error(f"interaction dependency cycle includes: {node}")
            return
        if node in visited:
            return
        visiting.add(node)
        for dependency in graph.get(node, []):
            visit(dependency)
        visiting.remove(node)
        visited.add(node)

    for node in graph:
        visit(node)


def validate(data: Any) -> Reporter:
    report = Reporter()
    if not isinstance(data, dict):
        report.error("the root value must be a JSON object")
        return report

    required = {
        "schema_version",
        "backend",
        "source_prompt",
        "assumptions",
        "open_questions",
        "scene",
        "assets",
        "interaction_points",
        "sensors",
        "task",
        "outputs",
    }
    for key in sorted(required - data.keys()):
        report.error(f"missing top-level field: {key}")

    if not isinstance(data.get("schema_version"), str) or not data["schema_version"].startswith("1."):
        report.error("schema_version must begin with '1.'")
    if data.get("backend") != "mujoco":
        report.error("backend must be 'mujoco'")
    if not isinstance(data.get("source_prompt"), str) or not data["source_prompt"].strip():
        report.error("source_prompt must be a non-empty string")
    check_string_list(data.get("assumptions"), "assumptions", report)
    check_string_list(data.get("open_questions"), "open_questions", report)

    scene = data.get("scene")
    if not isinstance(scene, dict):
        report.error("scene must be an object")
        scene = {}
    if not isinstance(scene.get("name"), str) or not scene.get("name", "").strip():
        report.error("scene.name must be a non-empty string")
    runtime = scene.get("runtime")
    if not isinstance(runtime, dict):
        report.error("scene.runtime must be an object")
        runtime = {}
    if not isinstance(runtime.get("mujoco_version"), str) or not runtime.get(
        "mujoco_version", ""
    ).strip():
        report.error("scene.runtime.mujoco_version must be a non-empty string")
    if runtime.get("mode") not in {"python", "viewer"}:
        report.error("scene.runtime.mode must be 'python' or 'viewer'")
    if not isinstance(runtime.get("headless"), bool):
        report.error("scene.runtime.headless must be a boolean")
    if runtime.get("gl_backend") not in ALLOWED_GL_BACKENDS:
        report.error(
            f"scene.runtime.gl_backend must be one of {sorted(ALLOWED_GL_BACKENDS)}"
        )

    world = scene.get("world")
    if not isinstance(world, dict):
        report.error("scene.world must be an object")
        world = {}
    if world.get("units") != "m":
        report.warning("scene.world.units is not 'm'; generated helpers assume meters")
    if world.get("up_axis") != "Z":
        report.warning("scene.world.up_axis is not 'Z'; MJCF conversion is required")
    check_vector(world.get("gravity"), 3, "scene.world.gravity", report)
    if not isinstance(world.get("ground"), bool):
        report.error("scene.world.ground must be a boolean")
    if not is_integer(world.get("seed")) or world.get("seed", -1) < 0:
        report.error("scene.world.seed must be an integer")

    namespaces = {object_type: set() for object_type in ALLOWED_TARGET_TYPES}
    assets, _ = check_unique_ids(data.get("assets"), "assets", report)
    for index, asset in enumerate(assets):
        if not isinstance(asset, dict):
            continue
        label = f"assets[{index}]"
        if asset.get("kind") not in ALLOWED_ASSET_KINDS:
            report.error(f"{label}.kind must be one of {sorted(ALLOWED_ASSET_KINDS)}")
        add_object_name(namespaces, "body", asset.get("body_name"), f"{label}.body_name", report)
        check_pose(asset.get("pose"), f"{label}.pose", report)

        geometry = asset.get("geometry")
        if not isinstance(geometry, dict):
            if asset.get("kind") in {"primitive", "mesh"}:
                report.error(f"{label}.geometry must be an object")
            geometry = {}
        if asset.get("kind") == "primitive":
            shape = geometry.get("shape")
            if shape not in ALLOWED_SHAPES:
                report.error(
                    f"{label}.geometry.shape must be one of {sorted(ALLOWED_SHAPES)}"
                )
            check_dimensions(
                geometry.get("dimensions"),
                shape,
                f"{label}.geometry.dimensions",
                report,
            )
            if shape == "open_box":
                check_bounds(
                    geometry.get("interior_bounds"),
                    f"{label}.geometry.interior_bounds",
                    report,
                )
                part_names = geometry.get("part_geom_names")
                if not isinstance(part_names, list) or len(part_names) < 5:
                    report.error(
                        f"{label}.geometry.part_geom_names must contain at least five names"
                    )
                else:
                    for part_index, name in enumerate(part_names):
                        add_object_name(
                            namespaces,
                            "geom",
                            name,
                            f"{label}.geometry.part_geom_names[{part_index}]",
                            report,
                        )
            else:
                add_object_name(
                    namespaces,
                    "geom",
                    geometry.get("geom_name"),
                    f"{label}.geometry.geom_name",
                    report,
                )
        elif asset.get("kind") == "mesh":
            source = asset.get("source") or geometry.get("mesh_path")
            if not isinstance(source, str) or not source.strip():
                report.error(f"{label}.source or geometry.mesh_path is required for mesh assets")
            add_object_name(
                namespaces,
                "geom",
                geometry.get("geom_name"),
                f"{label}.geometry.geom_name",
                report,
            )
        elif asset.get("kind") == "mjcf_include":
            source = asset.get("source") or asset.get("include_path")
            if not isinstance(source, str) or not source.strip():
                report.error(f"{label}.source or include_path is required for mjcf_include assets")
        elif asset.get("kind") == "robot":
            source = asset.get("source") or asset.get("model_path")
            if not isinstance(source, str) or not source.strip():
                report.error(f"{label}.source or model_path is required for robot assets")
        if asset.get("kind") in {"mjcf_include", "robot"}:
            exports = asset.get("exports", {})
            if not isinstance(exports, dict):
                report.error(f"{label}.exports must be an object when provided")
            else:
                for object_type, names in exports.items():
                    if object_type not in ALLOWED_TARGET_TYPES:
                        report.error(
                            f"{label}.exports has unsupported object type: {object_type}"
                        )
                        continue
                    if not isinstance(names, list):
                        report.error(f"{label}.exports.{object_type} must be a list")
                        continue
                    for name_index, name in enumerate(names):
                        add_object_name(
                            namespaces,
                            object_type,
                            name,
                            f"{label}.exports.{object_type}[{name_index}]",
                            report,
                        )

        physics = asset.get("physics")
        if not isinstance(physics, dict):
            report.error(f"{label}.physics must be an object")
            physics = {}
        if not isinstance(physics.get("dynamic"), bool):
            report.error(f"{label}.physics.dynamic must be a boolean")
        if "friction" in physics:
            if check_vector(physics["friction"], 3, f"{label}.physics.friction", report):
                if any(value < 0 for value in physics["friction"]):
                    report.error(f"{label}.physics.friction values must be non-negative")
        if "mass_kg" in physics and (
            not is_number(physics["mass_kg"]) or physics["mass_kg"] <= 0
        ):
            report.error(f"{label}.physics.mass_kg must be a positive finite number")
        for integer_field in ("contype", "conaffinity"):
            if integer_field in physics and (
                not is_integer(physics[integer_field]) or physics[integer_field] < 0
            ):
                report.error(f"{label}.physics.{integer_field} must be a non-negative integer")
        if "joint_name" in physics:
            add_object_name(
                namespaces, "joint", physics["joint_name"], f"{label}.physics.joint_name", report
            )
        elif physics.get("dynamic"):
            exported_joints = asset.get("exports", {}).get("joint", [])
            if asset.get("kind") not in {"mjcf_include", "robot"} or not exported_joints:
                report.error(f"{label}.physics.joint_name is required for a dynamic asset")
        if "actuator_name" in physics:
            add_object_name(
                namespaces,
                "actuator",
                physics["actuator_name"],
                f"{label}.physics.actuator_name",
                report,
            )

    sensors, _ = check_unique_ids(data.get("sensors"), "sensors", report)
    for index, sensor in enumerate(sensors):
        if not isinstance(sensor, dict):
            continue
        label = f"sensors[{index}]"
        if not isinstance(sensor.get("type"), str) or not sensor["type"].strip():
            report.error(f"{label}.type must be a non-empty string")
        if sensor.get("type") == "camera":
            add_object_name(
                namespaces, "camera", sensor.get("camera_name"), f"{label}.camera_name", report
            )
        frequency = sensor.get("frequency_hz")
        if not is_number(frequency) or frequency <= 0:
            report.error(f"{label}.frequency_hz must be positive")
        check_string_list(sensor.get("data"), f"{label}.data", report)
        if not sensor.get("data"):
            report.error(f"{label}.data must declare at least one observation")
        if "pose" in sensor:
            check_pose(sensor["pose"], f"{label}.pose", report)

    points, point_ids = check_unique_ids(
        data.get("interaction_points"), "interaction_points", report
    )
    for index, point in enumerate(points):
        if not isinstance(point, dict):
            continue
        label = f"interaction_points[{index}]"
        marker_site = point.get("marker_site")
        if marker_site is not None:
            add_object_name(namespaces, "site", marker_site, f"{label}.marker_site", report)
        if not isinstance(point.get("affordance"), str) or not point["affordance"].strip():
            report.error(f"{label}.affordance must be a non-empty string")
        check_pose(point.get("pose"), f"{label}.pose", report)
        action = point.get("action")
        if not isinstance(action, dict):
            report.error(f"{label}.action must be an object")
        else:
            if action.get("mode") not in ALLOWED_ACTION_MODES:
                report.error(f"{label}.action.mode must be one of {sorted(ALLOWED_ACTION_MODES)}")
            if not isinstance(action.get("command"), str) or not action.get("command", "").strip():
                report.error(f"{label}.action.command must be a non-empty string")
            if not isinstance(action.get("schema"), dict):
                report.error(f"{label}.action.schema must be an object")
        for field in ("preconditions", "success_conditions"):
            check_condition_list(point.get(field), f"{label}.{field}", report)
        check_string_list(point.get("effects"), f"{label}.effects", report)
        if not isinstance(point.get("reset"), dict):
            report.error(f"{label}.reset must be an object")

    check_dependencies(points, point_ids, report)

    for index, point in enumerate(points):
        if not isinstance(point, dict):
            continue
        target = point.get("target")
        label = f"interaction_points[{index}].target"
        if not isinstance(target, dict):
            report.error(f"{label} must be an object")
            continue
        object_type = target.get("type")
        name = target.get("name")
        if object_type not in ALLOWED_TARGET_TYPES:
            report.error(f"{label}.type must be one of {sorted(ALLOWED_TARGET_TYPES)}")
        elif not check_name(name, f"{label}.name", report):
            continue
        elif name not in namespaces[object_type]:
            report.error(f"{label} references unknown {object_type}: {name}")

    task = data.get("task")
    if not isinstance(task, dict):
        report.error("task must be an object")
    else:
        if not isinstance(task.get("goal"), str) or not task.get("goal", "").strip():
            report.error("task.goal must be a non-empty string")
        check_condition_list(task.get("success_conditions"), "task.success_conditions", report)
        check_condition_list(task.get("failure_conditions"), "task.failure_conditions", report)
        if not isinstance(task.get("reset_policy"), str) or not task.get(
            "reset_policy", ""
        ).strip():
            report.error("task.reset_policy must be a non-empty string")

    outputs = data.get("outputs")
    if not isinstance(outputs, dict):
        report.error("outputs must be an object")
    else:
        for flag, path_field in (
            ("save_mjcf", "mjcf_path"),
            ("save_mjb", "mjb_path"),
        ):
            if not isinstance(outputs.get(flag), bool):
                report.error(f"outputs.{flag} must be a boolean")
            if outputs.get(flag) and (
                not isinstance(outputs.get(path_field), str)
                or not outputs[path_field].strip()
            ):
                report.error(f"outputs.{path_field} is required when {flag} is true")
        if "resolution" in outputs:
            resolution = outputs["resolution"]
            if (
                not isinstance(resolution, list)
                or len(resolution) != 2
                or not all(is_integer(value) and value > 0 for value in resolution)
            ):
                report.error("outputs.resolution must be [positive_width, positive_height]")

    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", type=Path)
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)
    try:
        data = json.loads(args.spec.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    report = validate(data)
    result = {
        "status": "PASS" if not report.errors else "FAIL",
        "errors": report.errors,
        "warnings": report.warnings,
    }
    if args.as_json:
        print(json.dumps(result, indent=2))
    else:
        print(result["status"])
        for warning in report.warnings:
            print(f"WARNING: {warning}")
        for error in report.errors:
            print(f"ERROR: {error}")
    return 1 if report.errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
