# Validation Checklist

Report each layer independently as generated, executed, verified, failed, or skipped.

## Static

1. Validate `scene_spec.json` with the canonical skill validator.
2. Parse `model.xml` with an XML parser and compile Python sources.
3. Check manifest/spec parity, unique MuJoCo names by object type, typed target resolution, dependency existence, and dependency cycles.
4. Check geometry full-dimension to MJCF half-size conversion and `xyzw` to `wxyz` quaternion conversion.

## Physics Process

Run with `MUJOCO_GL=disable`. Compile the actual MJCF, construct `MjData`, locate all required objects by name, step the model, and reject non-finite state. Exercise invalid actions, the physical portion of a valid sequence, contact/settling predicates, and deterministic reset. Save and reopen requested XML/MJB artifacts.

## Render Process

Run in a new process with `MUJOCO_GL=egl` on Linux. If it fails because EGL/graphics is unavailable, record that failure and retry `MUJOCO_GL=osmesa` in another process. On macOS use MuJoCo's `mjpython` with `MUJOCO_GL=glfw` and an active graphics session; this selects the native CGL context. Capture before/after RGB and depth. Open the files/arrays and check dimensions, dynamic range, task-relevant pixels, finite geometry depth, and visible movement caused by the action sequence.

Do not claim GPU rendering when OSMesa was used. Do not treat a report screenshot or XML preview as a simulator screenshot; verify that image files came from the renderer.

## Failure Evidence

Record Python and MuJoCo versions, `MUJOCO_GL`, the exit code, and the first actionable failure category. Do not persist full command lines, local paths, hostnames, credentials, or traceback text; inspect local stderr for details. Distinguish package/import, MJCF compile, physics assertion, EGL/OSMesa setup, and image assertion failures.
