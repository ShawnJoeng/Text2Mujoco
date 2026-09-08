---
name: text2mujoco
description: "Convert a natural-language request into a runnable MuJoCo 3 environment with MJCF, physics, sensors, and executable interaction points. Use for creating or modifying MuJoCo scenes from text; do not use for Isaac Sim, generic 3D modeling, or rendering-only work."
---

# Text2MuJoCo

Turn a natural-language scene or task request into an inspectable MuJoCo package. The package must include a validated intermediate scene specification, loadable MJCF, a small Python environment API, and machine-readable interaction contracts. Generated files are not proof of a successful simulation; report runtime and render checks separately.

## Workflow

1. Parse the goal, entities, geometry, spatial relationships, dynamics, actuators, sensors, interaction affordances, success/failure predicates, reset behavior, runtime constraints, and requested outputs. Preserve the original request as `source_prompt`; record assumptions instead of silently inventing important details.
2. Ask only for blocking details. Otherwise default to SI meters, Z-up, gravity `[0, 0, -9.81]`, a ground plane, deterministic seed, primitive stand-ins, a fixed camera, and headless-capable execution.
3. Write `scene_spec.json` using [references/scene_spec.md](references/scene_spec.md), then run `scripts/validate_scene_spec.py` before producing MJCF or Python. Keep asset IDs and MuJoCo object names stable across edits.
4. Generate `model.xml` using structured XML APIs where practical. Read [references/mujoco_patterns.md](references/mujoco_patterns.md) for geometry conversion, quaternion order, dynamic bodies, rendering backends, and persistence rules. Do not use the deprecated `mujoco-py` package.
5. Implement every interaction point as an executable contract. Expose `list_interaction_points()`, `get_action_schema()`, `reset(seed=None)`, `step(action)`, `observe()`, and `is_success()` or an equivalent environment interface. Read [references/interaction_patterns.md](references/interaction_patterns.md) when implementing actions or markers.
6. Prefer locally available meshes and included MJCF assets. Do not fabricate unresolved paths or download assets without authorization. Use a tagged primitive fallback when it keeps the task executable.
7. Validate progressively using [references/validation_checklist.md](references/validation_checklist.md). Compile MJCF and run physics with `MUJOCO_GL=disable` before testing rendering. On macOS use the native CGL path (`MUJOCO_GL=glfw`); on Linux run EGL and OSMesa attempts in separate processes because the backend is selected when MuJoCo/OpenGL is first imported.

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

`environment.py` must load a caller-supplied model/spec path, avoid global mutable simulator state, and keep task logic independent of viewer keyboard timing. `model.xml` must compile with `mujoco.MjModel.from_xml_path`. Save `.mjb` only after a real model compile succeeds.

## Runtime Boundaries

- MuJoCo physics does not require an RTX GPU. Use `MUJOCO_GL=disable` for physics-only checks.
- Prefer `MUJOCO_GL=egl` on a Linux host with working EGL graphics exposure. If EGL is unavailable, try `MUJOCO_GL=osmesa` in a new process for CPU rendering. Do not label an OSMesa image as GPU-rendered.
- Keep `scene_spec.json` poses as `orientation_xyzw`; convert explicitly to MJCF `quat="w x y z"` and test the conversion.
- Scene dimensions are full metric dimensions. Convert boxes to MJCF half-sizes; convert cylinders/capsules to radius and half-length. Do not copy full dimensions directly into `geom size`.
- Use at least five colliders for an open box. A single box geom is solid and cannot represent a placement volume.
- A direct qpos change is acceptable only when the requested task uses a documented task-level abstraction. Use joints, actuators, contacts, tendons, or constraints when physical force/control behavior is required.

## Response

For Chinese requests, respond in Chinese unless asked otherwise. Summarize the normalized scene and assumptions, generated files, interaction IDs/action schemas, launch commands, and the exact evidence for static, physics, and rendering checks. Mark anything skipped or failed without implying it was verified.

When updating an existing package, preserve stable IDs, MuJoCo names, and public interaction methods unless the user explicitly accepts a breaking change. Patch the smallest relevant scope and rerun affected validation layers.

The scene spec is canonical input, but generated handlers are code: after changing interaction conditions, poses, targets, or task predicates, regenerate or patch the corresponding runtime handlers and rerun contract/behavior tests. Do not assume arbitrary condition strings are interpreted automatically.
