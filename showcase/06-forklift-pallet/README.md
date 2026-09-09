# Forklift Pallet Delivery

This MuJoCo 3.2.7 showcase demonstrates a mobile forklift delivering a loaded pallet around a storage rack. The orange forklift drives to the pallet, raises powered forks, engages the free pallet body, carries it through a marked route, lowers the forks into a green delivery zone, and verifies the result with a fixed RGB-D camera.

## Interaction sequence

`drive_to_pallet` -> `raise_forks` -> `engage_pallet` -> `carry_to_drop_zone` -> `lower_forks_release` -> `inspect_forklift_delivery`

The forklift has powered planar base slides, a steering hinge, and a vertical fork-lift slide. Once engaged, the pallet follows the fork anchor through the route while MuJoCo continues to advance physics. Release disables the hold, lowers the forks, and lets gravity and delivery-zone contacts settle the pallet. Every interaction has a visible named marker site and an explicit dependency.

The environment exposes `list_interaction_points()`, `get_action_schema()`, `reset(seed=None)`, `step({"id": "<interaction_id>", "payload": {}})`, `observe()`, and `is_success()`.

## Files

- `scene_spec.json`: normalized warehouse request, forklift/pallet assets, route, sensor, and interaction contract.
- `model.xml`: self-contained MJCF with mobile forklift joints, powered forks, free pallet, rack obstacle, delivery zone, markers, and camera.
- `environment.py`: action validation, deterministic route controller, pallet synchronization, RGB-D capture, reset, and success checks.
- `interaction_manifest.json`: canonical typed targets, dependency order, and marker mapping.
- `physics_smoke.py`: model, actuator, route, pallet-following, release, reset, and artifact reload checks.
- `render_smoke.py`: RGB-D validity, marker visibility, forklift motion, and reset checks.

## Run

From this directory:

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

On Linux, use `MUJOCO_GL=egl` or `MUJOCO_GL=osmesa` in a separate process. From the repository root, generate the dense 200 ms RGB GIF/TIFF with:

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene 06-forklift-pallet --dense-interval 0.20
```

All sensor and artifact paths are package-relative and reject absolute, traversal, URI, and symlink escapes.

Evidence: [`dense_sequence.gif`](output/screenshots/dense_sequence.gif) | [`dense_sequence.tif`](output/screenshots/dense_sequence.tif) | [`dense_sequence_results.json`](output/dense_sequence_results.json). Regular RGB frames are sampled every `0.20 s` of simulation time; the GIF uses `200 ms` per frame and `800 ms` for the final frame.
