# Text2MuJoCo verification report

Date: 2026-09-07

## Result

The generated MuJoCo package passed the complete static, physics, interaction, persistence, RGB, and depth test set. Physics was independently executed on the requested A100 iCoding host. Rendering was executed with the same MJCF and test code on the local Apple Silicon host because the remote container exposes neither a usable EGL device nor an OSMesa library. On macOS, `MUJOCO_GL=glfw` selects MuJoCo's native CGL context; it is not evidence of remote Linux EGL rendering.

## Evidence matrix

| Layer | Host/backend | Result | Evidence |
| --- | --- | --- | --- |
| Skill structure | local | PASS | `quick_validate.py` accepted `text2mujoco` |
| Scene specification | local | PASS | no errors or warnings |
| Validator negative cases | local | PASS | 17 invalid fixtures rejected; 4 legal variants accepted |
| MJCF/manifest contract | local | PASS | XML parse, typed targets, sizes, Python compile |
| MJCF runtime probe | local, MuJoCo 3.2.7, GL disabled | PASS | model compile, `MjData`, `mj_step`, finite state |
| Physics and interactions | A100 iCoding, MuJoCo 3.2.7, GL disabled (external terminal evidence) | PASS* | core actuator, gravity, 5-collider box contact, reset, MJCF/MJB reload |
| Physics and interactions | local, MuJoCo 3.2.7, GL disabled | PASS | same suite independently repeated |
| RGB-D renderer | local, MuJoCo 3.2.7, native CGL context (requested `MUJOCO_GL=glfw`) | PASS | two real `mujoco.Renderer` frames and depth arrays |
| Remote EGL renderer | A100 iCoding | FAIL (environment) | PyOpenGL could not resolve `eglQueryString` |
| Remote OSMesa renderer | A100 iCoding | FAIL (environment) | `/usr/lib/x86_64-linux-gnu/libOSMesa.so.8 -> /dev/null` |

## Physics assertions

- Stable named objects: 20.
- Open-box collision geoms: 5.
- Gravity: `[0.0, 0.0, -9.81]` m/s^2.
- Timestep: `0.002` s.
- Cube mass: `0.2` kg.
- Button slide joint reached the pressed range through its position actuator.
- Released cube settled at approximately `[0.30, 0.18, 0.84497]` m.
- Final linear speed was approximately `2.18e-14` m/s.
- Cube-to-box-bottom contact was present.
- Eleven invalid/dependency/payload/state-precondition/ID/limit branches raised `EnvironmentError`; four invalid reset seeds and four invalid step counts were rejected.
- Grasp hold, output flags, and declared marker/camera pose consistency passed.
- `press_start_button -> grasp_red_cube -> place_cube_in_box` passed.
- Custom/default seed reset, MJCF reload, and MJB reload passed.

The earlier remote core-physics result was observed in the A100 terminal at:

```text
/root/paddlejob/workspace/env_run/output/lzk/text2mujoco-test/output/physics_results.json
```

`*` The remote JSON is not copied into this local package, so that row is external/unverifiable from the repository alone. The final hardened source and all RGB-D evidence below were rerun locally.

## Render assertions

- Full sequence: `press_start_button -> grasp_red_cube -> place_cube_in_box -> inspect_rgbd`.
- RGB shape: `480 x 640 x 3` for both frames.
- Initial RGB standard deviation: `65.483`; final: `65.917`.
- Dynamic range: `255` for both frames.
- Red cube pixels: initial `1278`; final `1289`.
- Red cube centroid movement: `130.518` px.
- Depth shape: `480 x 640` for both frames.
- Finite geometry depth pixels (far-plane background excluded): `161229` for both frames.
- Observed geometry depth range: approximately `1.149` to `3.628` m; far plane is `16.0` m.
- Changed geometry depth pixels after interaction: `1815`.

Artifacts:

```text
output/physics_results.json
output/render_results.json
output/model.xml
output/model.mjb
output/screenshots/initial/rgb.png
output/screenshots/initial/depth.npy
output/screenshots/initial/depth_preview.png
output/screenshots/final/rgb.png
output/screenshots/final/depth.npy
output/screenshots/final/depth_preview.png
```

MuJoCo reported `ARB_clip_control unavailable while mjDEPTH_ZEROFAR requested` on the local macOS renderer. The test excludes far-plane background, uses broad validity/range checks rather than pixel-exact depth, and records the actual `mujoco.cgl.GLContext` separately from the requested `MUJOCO_GL=glfw` backend.
