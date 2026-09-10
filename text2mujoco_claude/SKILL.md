---
name: text2mujoco
description: This skill should be used when the user asks to "generate a MuJoCo environment from text", "create an MJCF scene", "add executable interaction points to a MuJoCo task", or modify and validate a MuJoCo simulation package described in natural language.
---

# Text2MuJoCo

Turn a natural-language scene or task request into an inspectable MuJoCo 3 package. Treat `$ARGUMENTS`, when present, as the user's scene request. Produce a validated intermediate scene specification, loadable MJCF, a small Python environment API, and machine-readable interaction contracts. Treat generated files as artifacts, not proof of a successful simulation; report runtime and render checks separately.

## Workflow

1. Parse the goal, entities, geometry, spatial relationships, dynamics, actuators, sensors, interaction affordances, success and failure predicates, reset behavior, runtime constraints, and requested outputs. Preserve the original request as `source_prompt`; record assumptions instead of silently inventing important details.
2. Ask only for blocking details. Otherwise default to SI meters, Z-up, gravity `[0, 0, -9.81]`, a ground plane, a deterministic seed, primitive stand-ins, a fixed camera, and headless-capable execution.
3. Write `scene_spec.json` using [references/scene_spec.md](references/scene_spec.md), then run `scripts/validate_scene_spec.py` before producing MJCF or Python. Keep asset IDs and MuJoCo object names stable across edits.
4. Generate `model.xml` using structured XML APIs where practical. Read [references/mujoco_patterns.md](references/mujoco_patterns.md) for geometry conversion, quaternion order, dynamic bodies, rendering backends, and persistence rules. Do not use the deprecated `mujoco-py` package.
5. Implement every interaction point as an executable contract and mirror it in the canonical `interaction_manifest.json`. Expose `list_interaction_points()`, `get_action_schema()` (a map from interaction ID to schema), `reset(seed=None)` (returning the initial observation), `step({"id": string, "payload": object})`, `observe()`, and `is_success()` or an equivalent environment interface. Read [references/interaction_patterns.md](references/interaction_patterns.md) before implementing actions or markers; read [references/robot_manipulation.md](references/robot_manipulation.md) when a robot, arm, gripper, conveyor, or synchronized object handoff is requested.
6. Prefer locally available meshes and included MJCF assets. Do not fabricate unresolved paths or download assets without authorization. Use a tagged primitive fallback when it keeps the task executable.
7. Validate progressively using [references/validation_checklist.md](references/validation_checklist.md). Compile MJCF and run physics with `MUJOCO_GL=disable` before testing rendering. Run the eight model-audit checks — collision geometry, declared mass, servo hold, link continuity, marker grounding, start pose, sequence contact, undeclared self-overlap — and when one fails, fix the scene rather than the threshold. On macOS, use MuJoCo's `mjpython` with an active graphics session and `MUJOCO_GL=glfw` for the native CGL path. On Linux, run EGL and OSMesa attempts in separate processes because the backend is selected when MuJoCo/OpenGL is first imported.
8. When the request includes a showcase or visual evidence, capture RGB-D keyframes after each dependency-ordered interaction and, when a dense sequence is requested, sample simulation time from the module-level `mujoco.mj_step` path. Record the interval and actual timestamps, verify nonblank RGB, and keep GIF/TIFF frame counts and timing consistent with the report.

Read [references/prompt_examples.md](references/prompt_examples.md) for compact manipulation, articulation, and navigation examples.

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

Make `environment.py` load caller-supplied model/spec paths, validate manifest/spec parity, avoid global mutable simulator state, and keep task logic independent of viewer keyboard timing. Compile `model.xml` with `mujoco.MjModel.from_xml_path`. Make `physics_smoke.py` audit initial contact for both the compiled `qpos0` and the post-`reset()` state, and fail when a contact reports `dist < -1e-4`; a reset that assigns positions can reintroduce overlap the MJCF does not have. Make it also wrap `mujoco.mj_step` for the whole documented sequence, report the deepest contact with its geom pair and time, and fail on overlap deeper than `-1e-3`. Save `.mjb` only after a real model compile succeeds. Keep persisted reports package-relative and free of hostnames, credentials, user paths, and raw tracebacks. Resolve capture output directories under the package root and reject escapes; artifact files must be written beneath a package subdirectory (typically `output/`), with symlink traversal rejected.

Before writing `source_prompt`, assumptions, or reports, remove credential-like values and machine-local paths from user text. Never persist API keys, access tokens, cookies, private keys, or full command lines; keep failure reports limited to an error type and a redacted diagnostic.

## Runtime Boundaries

- Run physics-only checks with `MUJOCO_GL=disable`; MuJoCo physics does not require an RTX GPU.
- Prefer `MUJOCO_GL=egl` on Linux with working EGL exposure. If EGL is unavailable, try `MUJOCO_GL=osmesa` in a new process as a CPU fallback. Do not label OSMesa output as GPU-rendered.
- Store poses in `scene_spec.json` as `orientation_xyzw`; convert explicitly to MJCF `quat="w x y z"` and test the conversion.
- Treat scene dimensions as full metric dimensions. Convert boxes to MJCF half-sizes and cylinders/capsules to radius and half-length.
- Use at least five colliders for an open box. A single box geom is solid and cannot represent a placement volume.
- Give every moving body collision geometry, and give the robot a boundary against itself. Robot links, gripper palms and fingers, mobile bases, masts, carriages, forks, wheels, pressed buttons, and pulled levers each need at least one geom with a nonzero `contype`/`conaffinity`. Put `conaffinity="7"` on every collision class so `robot<->robot` is tested too, then name each pair that is designed to nest in `<contact><exclude>`. `contype="0" conaffinity="0"` and a narrowed mask such as `contype="2" conaffinity="5"` both draw a body that passes through everything and report no contact, so neither may be used to suppress a pair. Allow a visual-only geom only for detail inside the collidable envelope of the same body. Verify the remaining overlaps with `mujoco.mj_geomDistance`, which ignores filtering: accept a welded neighbour or a declared `<exclude>`, reject anything else.
- Draw every prismatic axis. A slide joint whose travel carries no geometry opens a gap under the parent link that grows with the joint value and renders as the hand having fallen off the arm; model it as a sleeve on the parent plus a ram on the moving body that stays inside it at both end stops. Measure link continuity — the widest gap from each jointed body to its nearest drawn ancestor over the whole replay — and keep it under 5 mm.
- State `mass` on every geom of every jointed body. A geom with neither `mass` nor `density` compiles at density 1000, so a decorative hub can outweigh the arm.
- Keep every position servo holding `qpos0` within 2 mm and 1 degree. Steady-state droop is `weight / kp`; raise `kp` or add `gravcomp="1"` to every body the axis carries, leaf fingertips included. Do not move geometry down to meet a droop.
- Make markers decals, not floating balls. A `<site>` never generates contacts, so an emissive sphere hovering above a table is something every object visibly passes through. Draw each viewer-facing marker as a thin cylinder flush with the surface it annotates, over real geometry and off the payload's footprint; move sites the code reads as kinematic references to `group="4"`; delete sites nothing references.
- Carry a payload with `<equality><weld>` toggled through `data.eq_active`, writing `relpose` from the measured offset at the instant the jaws close so the constraint engages already satisfied. Close the jaws onto the payload, not around a gap. Release by deactivating the constraint and letting the payload settle. Prove it with a mid-air release that requires the payload to fall.
- Give the hand more freedom than the task strictly needs — typically a wrist pitch hinge with its own actuator — and exercise it in the scripted sequence, so setting a part down does not mean retracting the whole tool column back down the path it arrived on.
- Seat resting bodies on their supports with half-size arithmetic and verify that no geoms interpenetrate at `t=0`. A penetrating start pose usually settles during warmup and then passes every later check.
- Check overlapping static scenery by arithmetic. MuJoCo filters pairs whose bodies are both welded to the world, so a pedestal driven through a table slab reports no contact in any frame.
- Do not assume a motion is clear because both of its endpoints are. Interpolate the end-effector in Cartesian space, or declare `lift -> traverse -> lower` waypoints high enough that the carried object's lowest surface clears every rim in between. Audit contact on every step of the documented sequence, not only at `t=0`: a centimetre-deep overlap mid-transfer satisfies both endpoint tolerances.
- Use direct qpos changes only for a documented task-level abstraction. Use joints, actuators, contacts, tendons, or constraints when physical force/control behavior is required. A constraint does not excuse the path it drags the payload along, and a `qpos` write on the robot's own joints hides the tracking error the servo gains exist to control.

## Response

For Chinese requests, respond in Chinese unless asked otherwise. Summarize the normalized scene and assumptions, generated files, interaction IDs and action schemas, launch commands, and exact evidence for static, physics, and rendering checks. Mark anything skipped or failed without implying it was verified.

When updating an existing package, preserve stable IDs, MuJoCo names, and public interaction methods unless the user explicitly accepts a breaking change. After changing interaction conditions, poses, targets, or task predicates, regenerate or patch the matching runtime handlers and rerun affected contract and behavior tests. Do not assume arbitrary condition strings in the scene specification execute automatically.
