#!/usr/bin/env python3
"""Regression tests for the portable scene-spec validator."""

from __future__ import annotations

import copy
import unittest

from validate_scene_spec import validate


def base_spec() -> dict:
    return {
        "schema_version": "1.0",
        "backend": "mujoco",
        "source_prompt": "Make a small sample scene.",
        "assumptions": [],
        "open_questions": [],
        "scene": {
            "name": "validator_sample",
            "runtime": {
                "mujoco_version": "3.2.7",
                "mode": "python",
                "headless": True,
                "gl_backend": "disable",
            },
            "world": {
                "units": "m",
                "up_axis": "Z",
                "gravity": [0, 0, -9.81],
                "ground": True,
                "seed": 0,
            },
        },
        "assets": [
            {
                    "id": "sample_body",
                "kind": "primitive",
                    "body_name": "sample_body",
                "geometry": {
                    "shape": "box",
                    "dimensions": [0.2, 0.2, 0.2],
                    "geom_name": "sample_geom",
                },
                "pose": {
                    "position": [0, 0, 0.1],
                    "orientation_xyzw": [0, 0, 0, 1],
                },
                "physics": {"dynamic": False},
            }
        ],
        "interaction_points": [
            {
                "id": "inspect_sample",
                "target": {"type": "body", "name": "sample_body"},
                "affordance": "inspect",
                "pose": {"position": [0, 0, 0.1], "orientation_xyzw": [0, 0, 0, 1]},
                "action": {
                    "mode": "direct",
                    "command": "inspect_sample",
                    "schema": {
                        "type": "object",
                        "properties": {"enabled": {"type": "boolean"}},
                        "required": [],
                    },
                },
                "preconditions": [],
                "success_conditions": [],
                "depends_on": [],
                "effects": [],
                "reset": {},
            }
        ],
        "sensors": [],
        "task": {
            "goal": "Inspect the sample.",
            "success_conditions": [],
            "failure_conditions": [],
            "reset_policy": "mj_resetData_and_task_state",
        },
        "outputs": {"save_mjcf": False, "save_mjb": False},
    }


class ValidatorRegressionTests(unittest.TestCase):
    def assert_error(self, spec: dict, fragment: str) -> None:
        report = validate(spec)
        self.assertTrue(
            any(fragment in error for error in report.errors),
            f"expected {fragment!r}, got {report.errors!r}",
        )

    def test_valid_sample(self) -> None:
        report = validate(base_spec())
        self.assertEqual(report.errors, [])
        self.assertEqual(report.warnings, [])

    def test_unhashable_enum_values_are_reported(self) -> None:
        cases = (
            (("scene", "runtime", "mode"), [], "scene.runtime.mode"),
            (("scene", "runtime", "gl_backend"), {}, "scene.runtime.gl_backend"),
            (("assets", 0, "kind"), [], "assets[0].kind"),
            (("assets", 0, "geometry", "shape"), {}, "assets[0].geometry.shape"),
            (("interaction_points", 0, "action", "mode"), [], "interaction_points[0].action.mode"),
            (("interaction_points", 0, "target", "type"), {}, "interaction_points[0].target.type"),
        )
        for path, value, fragment in cases:
            spec = copy.deepcopy(base_spec())
            target = spec
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = value
            with self.subTest(path=path):
                self.assert_error(spec, fragment)

    def test_invalid_exports_does_not_crash(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["assets"] = [
            {
                "id": "sample_robot",
                "kind": "robot",
                "body_name": "sample_robot",
                "source": "robot.xml",
                "exports": None,
                "pose": {"position": [0, 0, 0]},
                "physics": {"dynamic": True},
            }
        ]
        self.assert_error(spec, "assets[0].exports must be an object")

    def test_invalid_action_schema_does_not_crash(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["interaction_points"][0]["action"]["schema"] = {
            "type": "object",
            "properties": None,
            "required": ["missing"],
        }
        self.assert_error(spec, "action.schema.properties must be an object")

    def test_huge_number_is_reported(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["scene"]["world"]["gravity"][0] = 10**1000
        self.assert_error(spec, "scene.world.gravity must be a list of 3 finite numbers")

    def test_units_and_axis_are_errors(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["scene"]["world"]["units"] = "ft"
        spec["scene"]["world"]["up_axis"] = "Y"
        report = validate(spec)
        self.assertTrue(any("units must be 'm'" in error for error in report.errors))
        self.assertTrue(any("up_axis must be 'Z'" in error for error in report.errors))
        self.assertEqual(report.warnings, [])

    def test_missing_orientation_is_reported(self) -> None:
        spec = copy.deepcopy(base_spec())
        del spec["assets"][0]["pose"]["orientation_xyzw"]
        self.assert_error(spec, "assets[0].pose.orientation_xyzw is required")

    def test_headless_glfw_is_reported(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["scene"]["runtime"]["gl_backend"] = "glfw"
        self.assert_error(spec, "headless cannot use glfw or cgl")

    def test_output_path_cannot_escape_package(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["outputs"]["save_mjcf"] = True
        spec["outputs"]["mjcf_path"] = "../outside/model.xml"
        self.assert_error(spec, "outputs.mjcf_path must stay inside the generated package")

    def test_windows_absolute_output_path_is_rejected(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["outputs"]["save_mjcf"] = True
        spec["outputs"]["mjcf_path"] = r"C:\\outside\\model.xml"
        self.assert_error(spec, "outputs.mjcf_path must stay inside the generated package")

    def test_nul_output_path_is_rejected(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["outputs"]["save_mjcf"] = True
        spec["outputs"]["mjcf_path"] = "model\x00.xml"
        self.assert_error(spec, "outputs.mjcf_path must stay inside the generated package")

    def test_schema_metadata_is_scanned_for_sensitive_text(self) -> None:
        spec = copy.deepcopy(base_spec())
        credential = "github" + "_pat_" + ("x" * 40)
        spec["interaction_points"][0]["action"]["schema"]["description"] = credential
        self.assert_error(spec, "contains a credential")

    def test_metadata_keys_are_scanned_for_sensitive_text(self) -> None:
        spec = copy.deepcopy(base_spec())
        credential_key = "github" + "_pat_" + ("x" * 40)
        spec["scene"][credential_key] = "redacted"
        self.assert_error(spec, "contains a credential")

    def test_sensitive_field_names_are_rejected(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["scene"]["api_key"] = "redacted"
        self.assert_error(spec, "scene.api_key contains a credential")

    def test_sensitive_prompt_text_is_rejected(self) -> None:
        spec = copy.deepcopy(base_spec())
        credential = "github" + "_pat_" + ("x" * 40)
        spec["source_prompt"] = "Use " + credential + "."
        self.assert_error(spec, "source_prompt contains a credential")

    def test_absolute_asset_reference_is_rejected(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["assets"][0]["kind"] = "mesh"
        local_path = "/" + "Users" + "/sample/mesh.obj"
        spec["assets"][0]["source"] = local_path
        spec["assets"][0]["geometry"] = {
            "shape": "box",
            "dimensions": [0.2, 0.2, 0.2],
            "geom_name": "sample_geom",
        }
        self.assert_error(spec, "assets[0].source must be a package-relative path")

    def test_external_asset_uri_is_rejected(self) -> None:
        spec = copy.deepcopy(base_spec())
        spec["assets"][0]["kind"] = "mesh"
        spec["assets"][0]["source"] = "https://example.invalid/mesh.obj"
        spec["assets"][0]["geometry"] = {
            "shape": "box",
            "dimensions": [0.2, 0.2, 0.2],
            "geom_name": "sample_geom",
        }
        self.assert_error(spec, "assets[0].source must be a package-relative path")


if __name__ == "__main__":
    unittest.main()
