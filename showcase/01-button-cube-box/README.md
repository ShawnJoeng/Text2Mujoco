# Text2MuJoCo end-to-end test

[English](README.md) · [中文](README.zh-CN.md)

This is the generated package for:

> Put a button, a red cube, and an open box on a table. After the button is pressed, grasp the cube and place it inside the box. Observe the result with an RGB-D camera.

The button uses a real slide joint and position actuator; its action uses `press_depth_m` in meters, matching the actuator's position target. The cube uses a free joint. Grasp is a `<weld>` equality constraint named `cube_grasp`, declared inactive and toggled at runtime; no robot was requested, so it anchors the cube to the world at the offset measured when it is picked up. Release deactivates the weld, after which gravity, contact, and settling are simulated by MuJoCo. The receptacle is five separate colliders rather than a solid box.

The checks in this directory target this axis-aligned button/cube/box example. A new package should regenerate its runtime handlers and checks for its own assets and interaction conditions. The scene uses the declared grasp pose and supplied placement height as the release target; the final resting pose is determined by physics. The first three interaction points are covered by the physics checks, and the inspection point is covered by the RGB-D render checks.

This test directory expects the sibling `../../text2mujoco_codex/scripts/validate_scene_spec.py`; run it in the repository layout shown here. Copying the test directory alone is not a self-contained skill installation.

## Run

Requires Python 3.9 or newer.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
./run_all.sh
```

`run_all.sh` first runs static and physics checks with `MUJOCO_GL=disable`. On macOS, automatic rendering uses MuJoCo's native CGL context through `MUJOCO_GL=glfw`. On Linux it tries EGL in one process and, if unavailable, OSMesa in a fresh process. Linux may need the corresponding runtime packages, commonly `libegl1` for EGL or `libosmesa6` for CPU rendering.

To choose explicitly:

```bash
MUJOCO_RENDER_BACKEND=osmesa ./run_all.sh
```

## Evidence

The suite verifies:

- scene specification validation and negative validator cases;
- manifest/spec parity and typed MuJoCo target resolution;
- MJCF XML parsing, model compilation, named bodies/geoms/joints/sites/camera;
- box/cylinder full-dimension conversion to MJCF sizes;
- button actuator motion, free-body gravity, five-part open-box contact, settling, and reset;
- MJCF and MJB reload;
- dependency/payload failure branches and the valid interaction sequence;
- real MuJoCo before/after RGB, depth arrays, image variation, red-cube visibility, and pixel movement.

Expected simulator images are `output/screenshots/initial/rgb.png` and `output/screenshots/final/rgb.png`. The project-wide collector writes a paced keyframe storyboard (`sequence.gif`, contact sheet, and `sequence.tif`) plus an optional dense RGB pass. The keyframe GIF is an animation of discrete verified states, not a render of every physics timestep; the TIFF is the full-resolution keyframe archive. Reports and XML previews are not simulator evidence; use renderer-produced image files.

To capture the dense pass from the repository root:

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene 01-button-cube-box
```

Dense RGB frames are sampled every `0.20 s` of MuJoCo simulation time by default. Use `--dense-interval <seconds>` to change that interval. `output/screenshots/dense_sequence.gif` plays ordinary frames with a `200 ms` delay (the final frame is `800 ms`), and `output/screenshots/dense_sequence.tif` stores the same full-resolution RGB pages. `output/dense_sequence_results.json` records the simulation timestamps and the one-based GIF/TIFF `archive_frame` mapping. Depth remains available in the verified keyframe capture under `output/screenshots/sequence/`.

## Verified runs (2026-09-08)

The checked-in physics and render reports record passing MJCF compilation, actuator motion, gravity release, open-box contact, finite state, failure branches, deterministic reset, RGB-D capture, visible cube movement, and the full four-action task. Re-run the commands above to reproduce the checks with the backend available on your machine. `model.xml` is the portable source; regenerate `model.mjb` for each target OS or architecture rather than copying a platform-specific binary cache.
