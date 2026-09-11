# Robotic Arm Sorting Cell

[English](README.md) · [中文](README.zh-CN.md)

This MuJoCo showcase is a compact industrial pick-and-place cell. A powered shoulder, elbow, and wrist arm approaches a blue cylindrical part on a conveyor, closes a parallel gripper, transfers the part over a blue receptacle, releases it, and verifies the sorted result with a fixed RGB-D camera. A red part and red receptacle provide a visible distractor lane.

## Interaction sequence

`approach_blue_part` -> `grasp_blue_part` -> `transfer_to_blue_bin` -> `release_blue_part` -> `inspect_sorting_result`

The first four actions are executable robot controls. The arm uses three hinge joints and position actuators; the two gripper fingers use powered slide joints. During the grasped phase the blue part is carried by the `blue_part_grasp` weld between `tool_turret` and the part, engaged from the offset measured at jaw close, while MuJoCo continues to advance physics. Release places the part above the open blue bin and runs settling steps before checking its local interior bounds. `inspect_sorting_result` requires an enabled MuJoCo rendering backend and writes RGB and depth arrays under the package directory.

The public API is `list_interaction_points()`, `get_action_schema()`, `reset(seed=None)`, `step({"id": "<interaction_id>", "payload": {}})`, `observe()`, and `is_success()`.

## Files

- `scene_spec.json`: normalized scene, robot assets, interaction dependencies, sensors, and output contract.
- `model.xml`: directly loadable MJCF with the articulated arm, powered gripper, parts, bins, table, conveyor, markers, and camera.
- `environment.py`: action validation, deterministic IK waypoints, gripper control, grasp synchronization, RGB-D capture, reset, and success checks.
- `interaction_manifest.json`: canonical typed interaction contract and dependency order.
- `physics_smoke.py`: model-name, actuator, IK, grasp/transfer/release, invalid-action, reset, and artifact reload checks.
- `render_smoke.py`: real camera RGB-D, marker visibility, object displacement, and final-state checks.

## Run

From this directory:

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

On Linux, use `MUJOCO_GL=egl` or `MUJOCO_GL=osmesa` in a separate process. From the repository root, the project capture collector can generate the paced keyframe and dense RGB archives once this scene is registered:

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --scene 05-robot-arm-sorting
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene 05-robot-arm-sorting --dense-interval 0.20
```

The dense archive samples simulation time every 0.20 seconds and stores the same RGB frames in a multi-page TIFF; the collector's report records the frame timestamps and archive indices.

Evidence: [`dense_sequence.gif`](output/screenshots/dense_sequence.gif) | [`dense_sequence.tif`](output/screenshots/dense_sequence.tif) | [`dense_sequence_results.json`](output/dense_sequence_results.json).
