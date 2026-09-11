# Conveyor-to-Arm Handoff Cell

[English](README.md) · [中文](README.zh-CN.md)

This MuJoCo 3.2.7 showcase demonstrates a synchronized conveyor and robot workflow. A powered roller advances a blue parcel to a pickup point, an articulated orange arm closes its parallel gripper around the parcel, transports it to a green target bin, releases it, and verifies the handoff with a fixed RGB-D camera. A red bin remains as a visible distractor.

## Interaction sequence

1. `start_conveyor_to_pickup` drives the conveyor roller while the parcel advances to the pickup index.
2. `move_arm_to_parcel` moves the arm joints and tool lift to the handoff pose.
3. `grasp_parcel_with_arm` closes both powered finger slides and engages the `parcel_grasp` weld from the offset measured at jaw close.
4. `move_arm_to_target_bin` lifts the held parcel above the bin rim, traverses in straight Cartesian sub-steps, then lowers it into the bin with the arm actuators.
5. `release_parcel_in_target_bin` opens both fingers, disables the hold, and settles the parcel in the five-piece target bin.
6. `inspect_handoff` captures RGB-D evidence from `handoff_camera`.

The environment exposes `list_interaction_points()`, `get_action_schema()`, `reset(seed=None)`, `step({"id": "<interaction_id>", "payload": {}})`, `observe()`, and `is_success()`. Every interaction has a visible named marker site.

## Run

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

Use `MUJOCO_GL=egl` or `MUJOCO_GL=osmesa` in separate Linux processes. Capture output and saved model artifacts remain package-relative and reject symlink traversal.

Evidence: [`dense_sequence.gif`](output/screenshots/dense_sequence.gif) | [`dense_sequence.tif`](output/screenshots/dense_sequence.tif) | [`dense_sequence_results.json`](output/dense_sequence_results.json). Regular RGB frames are sampled every `0.20 s` of simulation time; the GIF uses `200 ms` per frame and `800 ms` for the final frame.

## Files

- `scene_spec.json`: conveyor, robot, parcel, bins, sensor, and interaction contract.
- `model.xml`: self-contained MJCF with powered conveyor hinge, articulated arm, dual gripper slides, free parcel, bins, and markers.
- `environment.py`: synchronized conveyor/arm runtime API and safe artifact persistence.
- `interaction_manifest.json`: canonical machine-readable dependency order and marker mapping.
- `physics_smoke.py`: conveyor delivery, joint/actuator, grasp transport, release, and reset checks.
- `render_smoke.py`: RGB-D validity, marker visibility, arm movement, and reset checks.
