# Smart Tool Cabinet (MuJoCo 3.2.7)

This generated package implements a three-step task: press the green button to unlock a desktop tool cabinet, pull the drawer forward by `0.22 m`, then capture RGB-D evidence from a fixed inspection camera.

## Scene and Interaction

- `press_unlock_button` drives `unlock_button_slide` to `0.012 m`. A yellow emissive site marks the unlock point.
- `pull_drawer_22cm` is available only after unlocking. A position actuator drives `drawer_slide` toward `0.22 m`; a cyan emissive site marks the handle.
- `inspect_open_drawer` requires the drawer position to be at least `0.218 m`. It captures `640 x 480` RGB-D from `fixed_inspection_camera`; a magenta site marks the inspection point.

The drawer uses a real slide joint, position actuators, and multiple collision geoms. Pulling is a task-level control command, not a robot gripper contact simulation. The scene uses only MJCF primitives and has no external mesh or download dependency.

The environment exposes `list_interaction_points()`, `get_action_schema()`, `reset(seed=None)`, `step(action)`, `observe()`, and `is_success()`.

## Files

- `scene_spec.json`: normalized scene, assumptions, sensors, and action contract.
- `model.xml`: directly loadable MJCF.
- `environment.py`: environment API without global simulator state.
- `interaction_manifest.json`: machine-readable interaction order and marker mapping.
- `physics_smoke.py`: compilation, dynamics, rejection branches, drawer travel, reset, and XML/MJB reload checks.
- `render_smoke.py`: fixed-camera RGB-D, marker visibility, and handle-pixel movement checks.
- `output/sequence_results.json`: full storyboard capture report.

## Run

From this directory:

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

On macOS, `MUJOCO_GL=glfw` uses MuJoCo's native CGL context and `mjpython` connects the process to the graphics session. On Linux, try `MUJOCO_GL=egl` and then `MUJOCO_GL=osmesa` in separate processes.

The project-level collector writes one RGB-D keyframe after each interaction, a paced `sequence.gif`, a contact-sheet PNG, and a multi-page TIFF archive:

```bash
MUJOCO_GL=glfw mjpython ../../showcase/capture_sequences.py --scene 02-smart-drawer
```

The GIF keeps each keyframe visible for `1.6 s` and the final state for `2.6 s`. It is a readable animation of discrete interaction keyframes, not a frame-by-frame physics recording. The TIFF stores the same keyframes at full resolution but does not define a universal viewer playback speed.

## Evidence

Static, physics, and render reports are saved as `output/scene_spec_validation.json`, `output/physics_results.json`, and `output/render_results.json`. Only a render report with `status: PASS` counts as evidence that the camera, visible markers, drawer movement, and task success were actually verified.
