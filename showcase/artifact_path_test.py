#!/usr/bin/env python3
"""Regression tests for the generated environments' artifact path boundary."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path


def discover_scene_files(showcase_root: Path) -> tuple[Path, ...]:
    """Discover every numbered showcase environment without a manual allowlist."""
    return tuple(
        sorted(
            path.relative_to(showcase_root)
            for path in showcase_root.glob("0*/environment.py")
            if path.parent.is_dir()
        )
    )


def load_environment_module(path: Path, index: int) -> types.ModuleType:
    """Load only the path helper without requiring the MuJoCo wheel."""
    fake_mujoco = types.ModuleType("mujoco")
    fake_mujoco.mjtObj = types.SimpleNamespace(
        mjOBJ_BODY=1,
        mjOBJ_GEOM=2,
        mjOBJ_JOINT=3,
        mjOBJ_ACTUATOR=4,
        mjOBJ_SITE=5,
        mjOBJ_CAMERA=6,
    )
    previous = sys.modules.get("mujoco")
    sys.modules["mujoco"] = fake_mujoco
    try:
        module_spec = importlib.util.spec_from_file_location(
            f"artifact_path_environment_{index}", path
        )
        if module_spec is None or module_spec.loader is None:
            raise AssertionError(f"could not load {path}")
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
        return module
    finally:
        if previous is None:
            sys.modules.pop("mujoco", None)
        else:
            sys.modules["mujoco"] = previous


class ArtifactPathTests(unittest.TestCase):
    def test_all_environment_helpers_reject_escape_and_symlink_paths(self) -> None:
        showcase_root = Path(__file__).resolve().parent
        scene_files = discover_scene_files(showcase_root)
        self.assertGreaterEqual(len(scene_files), 1)
        for index, relative_path in enumerate(scene_files):
            with self.subTest(scene=relative_path.parent.name):
                module = load_environment_module(showcase_root / relative_path, index)
                environment_error = module.EnvironmentError
                helper = next(
                    value
                    for value in module.__dict__.values()
                    if isinstance(value, type)
                    and "_resolve_output_path" in value.__dict__
                )._resolve_output_path

                with tempfile.TemporaryDirectory(prefix="text2mujoco-artifact-") as temp_dir:
                    output_dir = Path(temp_dir) / "output"
                    output_dir.mkdir()
                    outside_dir = Path(temp_dir) / "outside"
                    outside_dir.mkdir()

                    self.assertEqual(
                        helper("./output/model.xml", output_dir),
                        (output_dir / "model.xml").resolve(),
                    )
                    for value in (
                        "../escape.xml",
                        "/tmp/escape.xml",
                        "https://example.invalid/escape.xml",
                        r"C:\\escape.xml",
                        r"C:escape.xml",
                        r"\\server\\share\\escape.xml",
                        ".",
                        None,
                    ):
                        with self.assertRaises(environment_error):
                            helper(value, output_dir)

                    try:
                        (output_dir / "parent").symlink_to(
                            outside_dir, target_is_directory=True
                        )
                        (output_dir / "target.xml").symlink_to(
                            outside_dir / "target.xml"
                        )
                    except (OSError, NotImplementedError):
                        # Some Windows environments disallow symlink creation;
                        # lexical boundary cases above remain portable.
                        continue
                    with self.assertRaises(environment_error):
                        helper("parent/model.xml", output_dir)

                    with self.assertRaises(environment_error):
                        helper("target.xml", output_dir)

    def test_save_artifacts_validates_all_targets_before_writing(self) -> None:
        showcase_root = Path(__file__).resolve().parent
        scene_files = discover_scene_files(showcase_root)
        self.assertGreaterEqual(len(scene_files), 1)
        for index, relative_path in enumerate(scene_files):
            with self.subTest(scene=relative_path.parent.name):
                module = load_environment_module(showcase_root / relative_path, index)
                environment_error = module.EnvironmentError
                environment_class = next(
                    value
                    for value in module.__dict__.values()
                    if isinstance(value, type) and "save_artifacts" in value.__dict__
                )
                module.mujoco.mj_saveModel = (
                    lambda _model, path, _options: Path(path).write_bytes(b"mjb")
                )

                with tempfile.TemporaryDirectory(prefix="text2mujoco-package-") as temp_dir:
                    package_root = Path(temp_dir).resolve()
                    model_path = package_root / "model.xml"
                    model_path.write_bytes(b"<mujoco/>")
                    output_dir = package_root / "output"
                    environment = environment_class.__new__(environment_class)
                    environment.package_root = package_root
                    environment.model_path = model_path
                    environment.model = object()
                    environment.spec = {
                        "outputs": {
                            "save_mjcf": True,
                            "save_mjb": True,
                            "mjcf_path": "./output/model.xml",
                            "mjb_path": "./output/model.mjb",
                        }
                    }
                    result = environment.save_artifacts(output_dir)
                    self.assertEqual(
                        result, {"mjcf": "output/model.xml", "mjb": "output/model.mjb"}
                    )

                    environment.spec["outputs"]["mjb_path"] = "./output/model.xml"
                    original_xml = (output_dir / "model.xml").read_bytes()
                    with self.assertRaises(environment_error):
                        environment.save_artifacts(output_dir)
                    self.assertEqual((output_dir / "model.xml").read_bytes(), original_xml)

                    environment.spec["outputs"]["mjb_path"] = "./output/model.mjb"
                    with self.assertRaises(environment_error):
                        environment.save_artifacts(package_root)

                    environment.spec["outputs"]["save_mjcf"] = "true"
                    with self.assertRaises(environment_error):
                        environment.save_artifacts(output_dir)


if __name__ == "__main__":
    unittest.main()
