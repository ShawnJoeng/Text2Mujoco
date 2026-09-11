# Robot Peg Assembly Cell

[English](README.md) · [中文](README.zh-CN.md)

This MuJoCo 3.2.7 showcase is a compact robot workcell: a planar orange arm lowers a gripper onto a red locating peg, transports it to a blue fixture, inserts it, releases it, and verifies the seated result with a fixed RGB-D camera.

## Interaction sequence

1. `move_arm_to_peg` moves three hinge joints to the peg approach pose.
2. `grasp_peg_with_arm` closes the actuated gripper and engages the `peg_grasp` weld from the offset measured at jaw close.
3. `move_arm_to_socket` transports the held peg with the arm actuators.
4. `insert_peg_into_socket` drives the tool lift slide joint down into the guide fixture.
5. `release_assembled_peg` opens the gripper and lets gravity/contact settle the peg in the socket.
6. `inspect_assembly` captures RGB-D evidence from `assembly_camera`.

All interaction points have named, non-colliding marker sites. The environment exposes `list_interaction_points()`, `get_action_schema()`, `reset(seed=None)`, `step({"id": "<interaction_id>", "payload": {}})`, `observe()`, and `is_success()`.

## Run

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

Use `MUJOCO_GL=egl` or `MUJOCO_GL=osmesa` for separate Linux rendering attempts. RGB-D captures are written below this package; artifact paths are package-scoped and reject symlink traversal.

Evidence: [`dense_sequence.gif`](output/screenshots/dense_sequence.gif) | [`dense_sequence.tif`](output/screenshots/dense_sequence.tif) | [`dense_sequence_results.json`](output/dense_sequence_results.json). Regular RGB frames are sampled every `0.20 s` of simulation time; the GIF uses `200 ms` per frame and `800 ms` for the final frame.

## Files

- `scene_spec.json`: normalized scene, robot exports, sensor, and interaction contract.
- `model.xml`: self-contained MJCF with explicit arm, tool, gripper, peg, fixture, joints, and actuators.
- `environment.py`: deterministic runtime API and safe artifact persistence.
- `interaction_manifest.json`: canonical machine-readable dependency order and marker mapping.
- `physics_smoke.py`: MJCF, joint, actuator, dependency, transport, release, and reset checks.
- `render_smoke.py`: RGB-D validity, marker visibility, arm movement, and reset checks.
