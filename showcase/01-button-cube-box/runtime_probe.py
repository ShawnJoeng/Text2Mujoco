#!/usr/bin/env python3
"""Compile and step the generated MJCF without creating a GL context."""

from __future__ import annotations

import argparse
import json
import os
import platform
from pathlib import Path

import mujoco
import numpy as np


def main() -> int:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=base / "model.xml")
    args = parser.parse_args()
    args.model = args.model.resolve()

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    for _ in range(20):
        mujoco.mj_step(model, data)
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
        raise AssertionError("runtime probe produced non-finite state")
    button = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "button_slide")
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "red_cube")
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overhead_rgbd")
    if min(button, cube, camera) < 0:
        raise AssertionError("runtime probe could not resolve required model names")
    print(
        json.dumps(
            {
                "status": "PASS",
                "mujoco_version": mujoco.__version__,
                "python_version": platform.python_version(),
                "path_base": "package_root",
                "mujoco_gl": os.environ.get("MUJOCO_GL", "unset"),
                "mjcf_compile": "PASS",
                "mjdata_create": "PASS",
                "mj_step": "PASS",
                "finite_state": "PASS",
                "nq": model.nq,
                "nv": model.nv,
                "ngeom": model.ngeom,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
