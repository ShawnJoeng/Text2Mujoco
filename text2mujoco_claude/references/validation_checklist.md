# Validation Checklist

Report each layer independently as generated, executed, verified, failed, or skipped.

## Static

1. Validate `scene_spec.json` with the canonical skill validator.
2. Parse `model.xml` with an XML parser and compile Python sources.
3. Check manifest/spec parity, unique MuJoCo names by object type, typed target resolution, dependency existence, and dependency cycles.
4. Check geometry full-dimension to MJCF half-size conversion and `xyzw` to `wxyz` quaternion conversion.
5. List every geom with `contype="0" conaffinity="0"` and every moving body with no collidable geom at all. A moving body without collision geometry fails this layer; a visual-only geom passes only when it lies inside the collidable envelope of the same body.

## Physics Process

Run with `MUJOCO_GL=disable`. Compile the actual MJCF, construct `MjData`, locate all required objects by name, step the model, and reject non-finite state. Exercise invalid actions, the physical portion of a valid sequence, contact/settling predicates, and deterministic reset. Save and reopen requested XML/MJB artifacts.

Audit initial contact after the first `mj_forward` and before any `mj_step`, for the compiled `qpos0` and again for the post-`reset()` state: reject the scene when a contact reports `dist < -1e-4` unless the spec declares that overlap as intentional pre-loaded contact. A reset that assigns positions from constants can reintroduce overlap the MJCF does not have. A penetrating start pose usually settles during warmup, so every later physics, render, and task assertion still passes; this is the only layer that catches it.

Then replay the documented sequence and audit contact on every step, not just at the ends. Wrap `mujoco.mj_step` for the whole run, record the deepest contact with the geom pair and the time it occurred, and reject overlap deeper than `-1e-3` outside of a declared impact. Sub-millimetre overlap under load is solver softness; a centimetre is a tool or payload passing through scenery, and the start pose, the end pose, and the endpoint tolerances are all still satisfied while it happens. Audit the same states the visual evidence samples, so a frame that is published cannot contain an overlap the report does not mention.

## Render Process

Run in a new process with `MUJOCO_GL=egl` on Linux. If it fails because EGL/graphics is unavailable, record that failure and retry `MUJOCO_GL=osmesa` in another process. On macOS use MuJoCo's `mjpython` with `MUJOCO_GL=glfw` and an active graphics session; this selects the native CGL context. Capture before/after RGB and depth. Open the files/arrays and check dimensions, dynamic range, task-relevant pixels, finite geometry depth, and visible movement caused by the action sequence.

Do not claim GPU rendering when OSMesa was used. Do not treat a report screenshot or XML preview as a simulator screenshot; verify that image files came from the renderer.

## Failure Evidence

Record Python and MuJoCo versions, `MUJOCO_GL`, the exit code, and the first actionable failure category. Do not persist full command lines, local paths, hostnames, credentials, or traceback text; inspect local stderr for details. Distinguish package/import, MJCF compile, physics assertion, EGL/OSMesa setup, and image assertion failures.
