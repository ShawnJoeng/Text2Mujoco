---
name: text2mujoco
description: "Convert a natural-language request into a runnable MuJoCo 3 environment with MJCF, physics, sensors, and executable interaction points. Use for creating or modifying MuJoCo scenes from text; do not use for other simulators, generic 3D modeling, or rendering-only work."
---

# Text2MuJoCo

Turn a natural-language scene or task request into an inspectable MuJoCo package. The package must include a validated intermediate scene specification, loadable MJCF, a small Python environment API, and machine-readable interaction contracts. Generated files are not proof of a successful simulation; report runtime and render checks separately.

## Workflow

1. Parse the goal, entities, geometry, spatial relationships, dynamics, actuators, sensors, interaction affordances, success/failure predicates, reset behavior, runtime constraints, and requested outputs. Preserve the original request as `source_prompt`; record assumptions instead of silently inventing important details.
2. Ask only for blocking details. Otherwise default to SI meters, Z-up, gravity `[0, 0, -9.81]`, a ground plane, deterministic seed, primitive stand-ins, a fixed camera, and headless-capable execution.
3. Write `scene_spec.json` using [references/scene_spec.md](references/scene_spec.md), then run `scripts/validate_scene_spec.py` before producing MJCF or Python. Keep asset IDs and MuJoCo object names stable across edits.
4. Generate `model.xml` using structured XML APIs where practical. Read [references/mujoco_patterns.md](references/mujoco_patterns.md) for geometry conversion, quaternion order, dynamic bodies, rendering backends, and persistence rules. Do not use the deprecated `mujoco-py` package.
5. Implement every interaction point as an executable contract and mirror it in the canonical `interaction_manifest.json`. Expose `list_interaction_points()`, `get_action_schema()` (a map from interaction ID to schema), `reset(seed=None)` (returning the initial observation), `step({"id": string, "payload": object})`, `observe()`, and `is_success()` or an equivalent environment interface. Read [references/interaction_patterns.md](references/interaction_patterns.md) when implementing actions or markers; read [references/robot_manipulation.md](references/robot_manipulation.md) when a robot, arm, gripper, conveyor, or synchronized object handoff is requested.
6. Prefer locally available meshes and included MJCF assets. Do not fabricate unresolved paths or download assets without authorization. Use a tagged primitive fallback when it keeps the task executable.
7. Validate progressively using [references/validation_checklist.md](references/validation_checklist.md). Compile MJCF and run physics with `MUJOCO_GL=disable` before testing rendering. On macOS use MuJoCo's `mjpython` with an active graphics session and the native CGL path (`MUJOCO_GL=glfw`); on Linux run EGL and OSMesa attempts in separate processes because the backend is selected when MuJoCo/OpenGL is first imported.
8. When the request includes a showcase or visual evidence, capture RGB-D keyframes after each dependency-ordered interaction and, when a dense sequence is requested, sample simulation time from the module-level `mujoco.mj_step` path. Record the interval and actual timestamps, verify nonblank RGB, and keep GIF/TIFF frame counts and timing consistent with the report.

See [references/prompt_examples.md](references/prompt_examples.md) for compact manipulation, articulation, and navigation requests.

## Output Package

Unless the user specifies another layout, create:

```text
scene_spec.json
model.xml
environment.py
interaction_manifest.json
physics_smoke.py
render_smoke.py
README.md
```

`environment.py` must load a caller-supplied model/spec path, validate manifest/spec parity, avoid global mutable simulator state, and keep task logic independent of viewer keyboard timing. `model.xml` must compile with `mujoco.MjModel.from_xml_path`. `physics_smoke.py` must audit initial contact for both the compiled `qpos0` and the post-`reset()` state and fail when a contact reports `dist < -1e-4`; a reset that assigns positions can reintroduce overlap the MJCF does not have. It must also wrap `mujoco.mj_step` for the whole documented sequence, report the deepest contact with its geom pair and time, and fail on overlap deeper than `-1e-3`. Save `.mjb` only after a real model compile succeeds. Keep persisted reports package-relative and free of hostnames, credentials, user paths, and raw tracebacks. Resolve capture output directories under the package root and reject escapes; artifact files must be written beneath a package subdirectory (typically `output/`), with symlink traversal rejected.

Before writing `source_prompt`, assumptions, or reports, remove credential-like values and machine-local paths from user text. Never persist API keys, access tokens, cookies, private keys, or full command lines; keep failure reports limited to an error type and a redacted diagnostic.

## Runtime Boundaries

- MuJoCo physics does not require an RTX GPU. Use `MUJOCO_GL=disable` for physics-only checks.
- Prefer `MUJOCO_GL=egl` on a Linux host with working EGL graphics exposure. If EGL is unavailable, try `MUJOCO_GL=osmesa` in a new process for CPU rendering. Do not label an OSMesa image as GPU-rendered.
- Keep `scene_spec.json` poses as `orientation_xyzw`; convert explicitly to MJCF `quat="w x y z"` and test the conversion.
- Scene dimensions are full metric dimensions. Convert boxes to MJCF half-sizes; convert cylinders/capsules to radius and half-length. Do not copy full dimensions directly into `geom size`.
- Use at least five colliders for an open box. A single box geom is solid and cannot represent a placement volume.
- Every moving body collides. Robot links, gripper palms and fingers, mobile bases, masts, carriages, forks, wheels, pressed buttons, and pulled levers each need at least one geom with a nonzero `contype`/`conaffinity`. `contype="0" conaffinity="0"` draws a body that passes through everything and reports no contact, so it is never the way to suppress an unwanted pair; use collision classes or `<contact><exclude>` and say which pair is excluded and why. A visual-only geom is acceptable only for detail inside the collidable envelope of the same body.
- Resting bodies must be seated on their supports by half-size arithmetic, with no geom interpenetration at `t=0`. A penetrating start pose usually settles during warmup and then passes every later check.
- Overlapping static scenery reports no contact at all, because MuJoCo filters pairs whose bodies are both welded to the world. Check pedestals, columns, and slabs that share `x, y` by arithmetic.
- A tool pose that is clear at both ends of a motion does not make the motion clear. Interpolate the end-effector in Cartesian space, or declare `lift -> traverse -> lower` waypoints high enough that the carried object's lowest surface clears every rim in between. Audit contact on every step of the documented sequence, not only at `t=0`: a centimetre-deep overlap in the middle of a transfer satisfies both endpoint tolerances.
- A direct qpos change is acceptable only when the requested task uses a documented task-level abstraction. Use joints, actuators, contacts, tendons, or constraints when physical force/control behavior is required. A pin does not excuse the path it drags the payload along.

## Response

For Chinese requests, respond in Chinese unless asked otherwise. Summarize the normalized scene and assumptions, generated files, interaction IDs/action schemas, launch commands, and the exact evidence for static, physics, and rendering checks. Mark anything skipped or failed without implying it was verified.

When updating an existing package, preserve stable IDs, MuJoCo names, and public interaction methods unless the user explicitly accepts a breaking change. Patch the smallest relevant scope and rerun affected validation layers.

The scene spec is canonical input, but generated handlers are code: after changing interaction conditions, poses, targets, or task predicates, regenerate or patch the corresponding runtime handlers and rerun contract/behavior tests. Do not assume arbitrary condition strings are interpreted automatically.
