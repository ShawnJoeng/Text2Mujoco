# Lever and Ramp Ball Workbench

[English](README.md) · [中文](README.zh-CN.md)

This generated MuJoCo 3.2.7 scene models a workbench where a blue lever raises an amber gate, a purple ball rolls down a guarded ramp into a green target tray, and a fixed RGB-D camera verifies the result. All four interaction points have visible MJCF site markers and are declared in `interaction_manifest.json` with typed targets, dependencies, effects, and reset state.

## Interaction Sequence

`pull_blue_lever` -> `check_release_zone` -> `confirm_target_tray` -> `inspect_with_camera`

The lever and gate use real hinge/slide joints with position actuators. The ball uses a free joint; after release, MuJoCo gravity, collision, friction, rolling, and settling determine its motion. The release-zone and target-tray actions advance physics and check position and speed. `inspect_with_camera` persists RGB and depth observations. `reset(seed=None)` returns the initial observation and actions use the exact `{"id": ..., "payload": ...}` envelope.

## Files

- `scene_spec.json`: normalized scene contract and assumptions.
- `model.xml`: editable MJCF source.
- `environment.py`: executable interaction, observation, reset, and success API.
- `interaction_manifest.json`: canonical typed targets, marker sites, action schemas, and dependencies.
- `output/scene_spec_validation.json`: committed scene-spec validation result.
- `output/physics_results.json`: physics and interaction validation result.
- `output/render_results.json`: RGB-D rendering and storyboard validation result.
- `output/model.xml`: model copy produced from the declared output settings.
- `physics_smoke.py`: actuator, release, settling, contact, reset, and reload checks.
- `render_smoke.py`: backend-selected RGB-D, marker visibility, before/after change, and storyboard checks.
- `output/sequence_results.json`: complete keyframe capture report.
- `output/dense_sequence_results.json`: dense simulation-time capture report with GIF/TIFF frame mapping.

## Run

From this directory:

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

On macOS, use MuJoCo's `mjpython` trampoline with `MUJOCO_GL=glfw` so the native CGL context is available. On Linux, try `MUJOCO_GL=egl` and then `MUJOCO_GL=osmesa` in separate processes. The scene spec keeps the backend at `auto`; the selected backend comes from the process environment. The project-level collector can regenerate the complete storyboard from the repository root:

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --scene 04-lever-ball-ramp
```

The collector writes a paced `sequence.gif`, a contact-sheet PNG, and a multi-page TIFF. The GIF keeps each keyframe visible for `1.6 s` and the final state for `2.6 s`; it is a readable animation of discrete interaction keyframes, not a frame-by-frame physics recording. The TIFF is a full-resolution keyframe archive and does not define a universal viewer playback speed.

For a denser RGB animation sampled from simulation time, run:

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene 04-lever-ball-ramp
```

The dense pass samples the first post-step state at each `0.20 s` boundary by default; pass `--dense-interval <seconds>` to change it. It adds action-boundary event frames. `output/screenshots/dense_sequence.gif` uses a `200 ms` delay for ordinary frames (`800 ms` for the final frame), while `output/screenshots/dense_sequence.tif` keeps the full-resolution RGB pages. The report records timestamps and stable `archive_frame` indices; depth remains in the verified keyframe capture.

## Output

The scene writes `output/physics_results.json`, `output/render_results.json`, `output/sequence_results.json`, and `output/dense_sequence_results.json`, plus the keyframe and dense GIF/TIFF archives under `output/screenshots/`. Per-stage RGB-D arrays are stored under `output/screenshots/sequence/`. `scene_spec.json` is the canonical input and `model.xml` is the editable MJCF source.
