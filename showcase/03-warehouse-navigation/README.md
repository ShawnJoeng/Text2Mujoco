# Small Warehouse Navigation (MuJoCo 3.2.7)

This generated package implements an ordered warehouse-navigation task. An orange planar robot starts in the southwest corner, reaches yellow checkpoint A, follows three safe waypoints along the south side of the main shelves to green checkpoint B, and finally captures `640 x 480` RGB-D evidence with a fixed three-quarter overhead camera.

## Scene and Interaction

- `reach_checkpoint_a` drives from the start to `checkpoint_a_marker` with a final position error no greater than `0.10 m`.
- `reach_checkpoint_b` is available only after A. The controller passes through `[-1.25, -1.35]`, `[0.75, -1.35]`, and `[1.15, -0.95]` before reaching B.
- `inspect_top_camera` is available only after B and captures RGB-D from `top_camera`.

Yellow, green, and magenta collision-free sites mark A, B, and the camera checkpoint. The robot uses two orthogonal slide joints and position actuators for deterministic planar navigation. This is a task-level motion controller, not a differential-drive or tire-slip model. Shelf geoms are collidable box primitives, and the controller checks actual MuJoCo contacts during motion and settling.

The environment exposes `list_interaction_points()`, `get_action_schema()`, `reset(seed=None)` (returns the initial observation), `step({"id": "<interaction_id>", "payload": {}})`, `observe()`, and `is_success()`.

## Files

- `scene_spec.json`: normalized scene, assumptions, sensors, action dependencies, and task conditions.
- `model.xml`: directly loadable MJCF scene.
- `environment.py`: environment API, action validation, dependency checks, waypoint control, collision checks, and RGB-D capture.
- `interaction_manifest.json`: canonical machine-readable interaction points, visible markers, and action schemas.
- `physics_smoke.py`: MJCF compilation, dynamics, invalid actions, route safety, collision, reset, and XML/MJB reload validation.
- `render_smoke.py`: real overhead RGB-D, marker visibility, robot pixel movement, task success, and render-reset validation.
- `output/sequence_results.json`: full storyboard capture report.
- `output/dense_sequence_results.json`: dense simulation-time capture report with GIF/TIFF frame mapping.

## Run

From this directory:

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

On macOS, `MUJOCO_GL=glfw` uses MuJoCo's native CGL context and requires access to the current graphics session. On Linux, try EGL and then OSMesa in separate processes.

The project-level collector writes one RGB-D keyframe after each interaction, a paced `sequence.gif`, a contact-sheet PNG, and a multi-page TIFF archive:

```bash
MUJOCO_GL=glfw mjpython ../../showcase/capture_sequences.py --scene 03-warehouse-navigation
```

The GIF keeps each keyframe visible for `1.6 s` and the final state for `2.6 s`. It is a readable animation of discrete interaction keyframes, not a frame-by-frame physics recording. The TIFF stores the same keyframes at full resolution but does not define a universal viewer playback speed.

For a denser RGB animation sampled from simulation time, run:

```bash
MUJOCO_GL=glfw mjpython ../../showcase/capture_sequences.py --dense --scene 03-warehouse-navigation
```

The dense pass samples the first post-step state at each `0.20 s` boundary by default; pass `--dense-interval <seconds>` to change it. It adds action-boundary event frames. `output/screenshots/dense_sequence.gif` uses a `200 ms` delay for ordinary frames (`800 ms` for the final frame), while `output/screenshots/dense_sequence.tif` keeps the full-resolution RGB pages. The report records timestamps and stable `archive_frame` indices; depth remains in the verified keyframe capture.

## Evidence

Static, physics, and render reports are saved as `output/scene_spec_validation.json`, `output/physics_results.json`, and `output/render_results.json`. Only a render report with `status: PASS` counts as evidence that the screenshots came from MuJoCo, all three markers were visible, the robot moved safely, the task completed, and reset restored the initial state. The dense report additionally verifies archive frame counts, timing, and non-blank RGB frames.
