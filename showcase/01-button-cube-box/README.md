# Text2MuJoCo end-to-end test

This is the generated package for:

> Put a button, a red cube, and an open box on a table. After the button is pressed, grasp the cube and place it inside the box. Observe the result with an RGB-D camera.

The button uses a real slide joint and position actuator; its action uses `press_depth_m` in meters, matching the actuator's position target. The cube uses a free joint. Grasp is deliberately a task-level qpos abstraction because no robot was requested; while grasped, the generated environment holds the free-joint pose until release, after which gravity, contact, and settling are simulated by MuJoCo. The receptacle is five separate colliders rather than a solid box.

The fixture tests in this directory are intentionally tied to the named, axis-aligned button/cube/box example. The reusable `text2mujoco_codex` skill and validator support other asset names, optional markers, additional open-box colliders, and rotated models; a newly generated package must regenerate its runtime handlers and tests rather than reuse these fixture assertions. The generated fixture uses the declared grasp interaction pose and the supplied placement z as the release target, while the final resting pose is determined by physics. Manifest point statuses record evidence provenance: the first three points were exercised in both the remote physics run and local rerun; the inspect point was verified in the local CGL render run.

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

Expected simulator images are `output/screenshots/initial/rgb.png` and `output/screenshots/final/rgb.png`. The project-wide collector additionally writes a paced `output/screenshots/sequence.gif`, a contact sheet, and the multi-page `output/screenshots/sequence.tif`. The GIF is an animation of discrete verified keyframes, not a render of every physics timestep; the TIFF is the full-resolution keyframe archive. A report screenshot or synthetic image is not accepted as simulator evidence.

## Verified runs (2026-09-07)

The A100 iCoding terminal showed a passing physics run with MuJoCo 3.2.7 and `MUJOCO_GL=disable`: MJCF compilation, the button actuator, gravity release, open-box contact, finite state, action failure branches, deterministic reset, and MJCF/MJB reload passed. This is external terminal evidence; the final hardened package was rerun locally, while the remote JSON itself is not included in this repository. It remains stored at:

```text
/root/paddlejob/workspace/env_run/output/lzk/text2mujoco-test/output/physics_results.json
```

The remote container cannot create an EGL context and intentionally maps `libOSMesa.so.8` to `/dev/null`, so it cannot produce an honest MuJoCo renderer screenshot without changing the container image. The exact same MJCF and tests were therefore run with MuJoCo 3.2.7 and the native CGL context on the local Apple Silicon host. RGB, depth, visible cube movement, and the full four-action task all passed. See `TEST_REPORT.md` for the evidence split. `model.xml` is the portable source; regenerate `model.mjb` on each target OS/architecture rather than copying the macOS binary cache to Linux.
