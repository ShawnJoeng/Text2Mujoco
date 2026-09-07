#!/usr/bin/env python3
"""Positive and negative tests for the Text2MuJoCo scene validator."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path


BASE = Path(__file__).resolve().parent
VALIDATOR_DIR = BASE.parents[1] / "text2mujoco" / "scripts"
sys.path.insert(0, str(VALIDATOR_DIR))

from validate_scene_spec import validate  # noqa: E402


def expect_error(spec: dict, fragment: str) -> None:
    errors = validate(spec).errors
    if not any(fragment in error for error in errors):
        raise AssertionError(f"expected error containing {fragment!r}, got {errors!r}")


def expect_valid(spec: dict, label: str) -> None:
    report = validate(spec)
    if report.errors:
        raise AssertionError(f"{label} should be valid, got {report.errors!r}")


def main() -> int:
    valid = json.loads((BASE / "scene_spec.json").read_text(encoding="utf-8"))
    report = validate(valid)
    if report.errors or report.warnings:
        raise AssertionError(
            f"valid fixture failed: errors={report.errors!r}, warnings={report.warnings!r}"
        )

    wrong_backend = copy.deepcopy(valid)
    wrong_backend["backend"] = "isaac"
    expect_error(wrong_backend, "backend must be 'mujoco'")

    duplicate_id = copy.deepcopy(valid)
    duplicate_id["assets"][1]["id"] = duplicate_id["assets"][0]["id"]
    expect_error(duplicate_id, "duplicate assets id")

    duplicate_body = copy.deepcopy(valid)
    duplicate_body["assets"][1]["body_name"] = duplicate_body["assets"][0]["body_name"]
    expect_error(duplicate_body, "duplicate MuJoCo body name")

    unknown_target = copy.deepcopy(valid)
    unknown_target["interaction_points"][0]["target"]["name"] = "missing_joint"
    expect_error(unknown_target, "references unknown joint")

    bad_cylinder = copy.deepcopy(valid)
    bad_cylinder["assets"][1]["geometry"]["dimensions"][1] = 0.07
    expect_error(bad_cylinder, "cylinder x/y diameters must match")

    bad_capsule = copy.deepcopy(valid)
    bad_capsule["assets"][1]["geometry"]["shape"] = "capsule"
    bad_capsule["assets"][1]["geometry"]["dimensions"] = [0.08, 0.06, 0.12]
    expect_error(bad_capsule, "capsule x/y diameters must match")

    missing_joint = copy.deepcopy(valid)
    del missing_joint["assets"][2]["physics"]["joint_name"]
    expect_error(missing_joint, "joint_name is required for a dynamic asset")

    zero_quaternion = copy.deepcopy(valid)
    zero_quaternion["assets"][0]["pose"]["orientation_xyzw"] = [0, 0, 0, 0]
    expect_error(zero_quaternion, "must not be zero")

    bad_bounds = copy.deepcopy(valid)
    bad_bounds["assets"][3]["geometry"]["interior_bounds"]["min"][0] = 0.2
    expect_error(bad_bounds, "min must be less than max")

    dependency_cycle = copy.deepcopy(valid)
    dependency_cycle["interaction_points"][0]["depends_on"] = ["inspect_rgbd"]
    dependency_errors = validate(dependency_cycle).errors
    if not any(
        fragment in error
        for fragment in ("interaction dependency cycle", "must reference an earlier point")
        for error in dependency_errors
    ):
        raise AssertionError(
            f"expected dependency graph rejection, got {dependency_errors!r}"
        )

    empty_sensor_data = copy.deepcopy(valid)
    empty_sensor_data["sensors"][0]["data"] = []
    expect_error(empty_sensor_data, "must declare at least one observation")

    missing_output = copy.deepcopy(valid)
    del missing_output["outputs"]["mjb_path"]
    expect_error(missing_output, "mjb_path is required")

    boolean_seed = copy.deepcopy(valid)
    boolean_seed["scene"]["world"]["seed"] = True
    expect_error(boolean_seed, "seed must be an integer")

    negative_seed = copy.deepcopy(valid)
    negative_seed["scene"]["world"]["seed"] = -1
    expect_error(negative_seed, "seed must be an integer")

    boolean_contype = copy.deepcopy(valid)
    boolean_contype["assets"][0]["physics"]["contype"] = True
    expect_error(boolean_contype, "contype must be a non-negative integer")

    bad_mass = copy.deepcopy(valid)
    bad_mass["assets"][2]["physics"]["mass_kg"] = 0
    expect_error(bad_mass, "mass_kg must be a positive finite number")

    boolean_resolution = copy.deepcopy(valid)
    boolean_resolution["outputs"]["resolution"][0] = True
    expect_error(boolean_resolution, "resolution must be")

    mesh_asset = copy.deepcopy(valid)
    mesh_asset["assets"][0]["kind"] = "mesh"
    mesh_asset["assets"][0]["source"] = "./assets/table.obj"
    mesh_asset["assets"][0]["geometry"] = {"geom_name": "table_geom"}
    expect_valid(mesh_asset, "mesh asset")

    optional_marker = copy.deepcopy(valid)
    del optional_marker["interaction_points"][3]["marker_site"]
    expect_valid(optional_marker, "optional marker")

    six_part_box = copy.deepcopy(valid)
    six_part_box["assets"][3]["geometry"]["part_geom_names"].append("box_lip")
    expect_valid(six_part_box, "six-part open box")

    include_asset = copy.deepcopy(valid)
    include_asset["assets"][0]["kind"] = "mjcf_include"
    include_asset["assets"][0]["source"] = "./assets/table.xml"
    include_asset["assets"][0].pop("geometry")
    expect_valid(include_asset, "MJCF include without geometry")

    forward_dependency = copy.deepcopy(valid)
    forward_dependency["interaction_points"][0]["depends_on"] = ["grasp_red_cube"]
    expect_error(forward_dependency, "must reference an earlier point")

    result = {
        "status": "PASS",
        "valid_fixture": "PASS",
        "negative_cases": 17,
        "positive_variants": 4,
    }
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
