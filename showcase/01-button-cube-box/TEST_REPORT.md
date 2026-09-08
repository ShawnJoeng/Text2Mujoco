# Text2MuJoCo verification report

Date: 2026-09-09

## Result

The generated MuJoCo package passed the static, physics, interaction, persistence, RGB, and depth checks recorded in this repository. The report describes only artifacts that can be reproduced from the commands and files in this directory. Rendering uses the backend selected by `MUJOCO_GL`; on macOS, `MUJOCO_GL=glfw` selects MuJoCo's native CGL context.

## Evidence matrix

| Layer | Configuration | Result | Evidence |
| --- | --- | --- | --- |
| Skill structure | repository layout | PASS | required `SKILL.md`, references, and validator are present |
| Scene specification | validator | PASS | no errors or warnings |
| Validator negative cases | validator | PASS | 17 invalid scene variants rejected; 4 legal variants accepted |
| MJCF/manifest contract | static checks | PASS | XML parse, typed targets, sizes, Python compile |
| MJCF runtime probe | MuJoCo 3.2.7, GL disabled | PASS | model compile, `MjData`, `mj_step`, finite state |
| Physics and interactions | MuJoCo 3.2.7, GL disabled | PASS | actuator, gravity, 5-collider box contact, reset, MJCF/MJB reload |
| RGB-D renderer | MuJoCo 3.2.7, requested `MUJOCO_GL=glfw` | PASS | two real `mujoco.Renderer` frames and depth arrays |

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

MuJoCo may report `ARB_clip_control unavailable while mjDEPTH_ZEROFAR requested` on systems without that extension. The test excludes far-plane background, uses broad validity/range checks rather than pixel-exact depth, and records the actual renderer context separately from the requested `MUJOCO_GL=glfw` backend.
