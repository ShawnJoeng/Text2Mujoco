#!/usr/bin/env python3
"""Regression tests for the portable scene-spec validator."""

from __future__ import annotations

import copy
import contextlib
import io
import unittest

from validate_scene_spec import main, validate


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
        report = validate(spec)
        rendered = "\n".join(report.errors)
        self.assertIn("contains a credential", rendered)
        self.assertNotIn(credential_key, rendered)

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

    def test_common_machine_paths_are_rejected_without_echoing(self) -> None:
        paths = (
            "/tmp/private-scene",
            "/private/tmp/private-scene",
            "/var/tmp/private-scene",
            "/private/var/folders/private-scene",
            "~/private-scene",
        )
        for path in paths:
            spec = copy.deepcopy(base_spec())
            spec["metadata"] = {"note": path}
            report = validate(spec)
            rendered = "\n".join(report.errors)
            self.assertTrue(report.errors, path)
            self.assertNotIn(path, rendered)

    def test_prompt_and_condition_urls_are_rejected_without_echoing(self) -> None:
        secret_url = "https://example.invalid/?token=github" + "_pat_" + ("x" * 40)
        spec = copy.deepcopy(base_spec())
        spec["source_prompt"] = "Use " + secret_url
        secret_key = "github" + "_pat_" + ("y" * 40)
        spec["interaction_points"][0]["preconditions"] = [
            {"op": "eq", "path": secret_url},
            {secret_key: secret_url},
        ]
        report = validate(spec)
        rendered = "\n".join(report.errors)
        self.assertTrue(report.errors)
        self.assertNotIn(secret_url, rendered)
        self.assertNotIn(secret_key, rendered)

    def test_unknown_ids_are_redacted_in_errors(self) -> None:
        unknown_target = "private_body_identifier"
        unknown_dependency = "private_dependency_identifier"
        spec = copy.deepcopy(base_spec())
        spec["interaction_points"][0]["target"]["name"] = unknown_target
        spec["interaction_points"][0]["depends_on"] = [unknown_dependency]
        report = validate(spec)
        rendered = "\n".join(report.errors)
        self.assertIn("unknown body: <redacted>", rendered)
        self.assertIn("unknown id: <redacted>", rendered)
        self.assertNotIn(unknown_target, rendered)
        self.assertNotIn(unknown_dependency, rendered)

    def test_sensitive_schema_field_is_not_echoed(self) -> None:
        spec = copy.deepcopy(base_spec())
        secret_field = "github" + "_pat_" + ("x" * 40)
        spec["interaction_points"][0]["action"]["schema"]["properties"] = {
            secret_field: {"type": "invalid"}
        }
        report = validate(spec)
        rendered = "\n".join(report.errors)
        self.assertTrue(report.errors)
        self.assertNotIn(secret_field, rendered)

    def test_common_credential_formats_are_rejected_without_echoing(self) -> None:
        values = (
            "AKIAIOSFODNN7EXAMPLE",
            "AIza" + ("A" * 35),
            "xoxb-" + ("x" * 20),
            "eyJ" + ("a" * 12) + "." + ("b" * 12) + "." + ("c" * 12),
            "token=" + ("x" * 16),
            "$HOME/.ssh/id_rsa",
        )
        for value in values:
            with self.subTest(value=value):
                spec = copy.deepcopy(base_spec())
                spec["metadata"] = {"value": value}
                report = validate(spec)
                rendered = "\n".join(report.errors)
                self.assertTrue(report.errors)
                self.assertNotIn(value, rendered)

    def test_descriptive_urls_without_credentials_are_allowed(self) -> None:
        spec = base_spec()
        spec["source_prompt"] = "See https://example.com/docs for context."
        self.assertEqual(validate(spec).errors, [])

    def test_read_error_does_not_echo_spec_path(self) -> None:
        path = "/Users/private/" + "github" + "_pat_" + ("x" * 40) + ".json"
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            code = main([path, "--json"])
        self.assertEqual(code, 2)
        self.assertIn("could not read scene spec", stderr.getvalue())
        self.assertNotIn(path, stderr.getvalue())

    def test_mjcf_and_mjb_output_paths_must_differ(self) -> None:
        spec = base_spec()
        spec["outputs"] = {
            "save_mjcf": True,
            "save_mjb": True,
            "mjcf_path": "./output/model.xml",
            "mjb_path": "output/model.xml",
        }
        self.assert_error(spec, "outputs.mjcf_path and outputs.mjb_path must differ")

    def test_extra_output_paths_are_package_relative(self) -> None:
        spec = base_spec()
        spec["outputs"]["screenshots"] = ["/tmp/private.png"]
        self.assert_error(spec, "outputs.screenshots[0] must be a package-relative path")
        spec["outputs"]["screenshots"] = [None]
        self.assert_error(
            spec, "outputs.screenshots[0] must be a non-empty package-relative path"
        )
        spec["outputs"] = {
            "save_mjcf": True,
            "save_mjb": False,
            "mjcf_path": "https://example.invalid/model.xml",
        }
        self.assert_error(spec, "outputs.mjcf_path must stay inside the generated package")

        spec["outputs"]["mjcf_path"] = "."
        self.assert_error(spec, "outputs.mjcf_path must name a file")


if __name__ == "__main__":
    unittest.main()
