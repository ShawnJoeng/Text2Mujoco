# Text2MuJoCo

<p align="center">
  <strong>Generate runnable MuJoCo environments from natural language.</strong><br>
  From one prompt to validated MJCF, executable interaction points, and real RGB-D evidence.
</p>

<p align="center">
  <img alt="MuJoCo 3.2.7" src="https://img.shields.io/badge/MuJoCo-3.2.7-2f6f61">
  <img alt="Python 3.9+" src="https://img.shields.io/badge/Python-3.9%2B-3776ab">
  <a href="LICENSE"><img alt="MIT License" src="https://img.shields.io/badge/License-MIT-6b7280"></a>
</p>

<p align="center">
  <a href="#showcase">Showcase</a> |
  <a href="#usage">Usage</a> |
  <a href="text2mujoco_codex/README.md">Codex</a> |
  <a href="text2mujoco_claude/README.md">Claude Code</a>
</p>

Text2MuJoCo turns a scene or task description into a loadable MuJoCo package. It resolves objects, physics, sensors, action order, success conditions, and visible interaction points, then validates the result with machine-readable reports.

Use the [Codex skill](text2mujoco_codex/README.md) or the [Claude Code skill](text2mujoco_claude/README.md).

## Showcase

### 01 / Button, Cube, and Box

> **Prompt**
>
> Place a button, a red cube, and an open box on a table. Press the button, grasp the cube, place it in the box, and inspect the result with an RGB-D camera.

**Interaction** - `press_start_button` -> `grasp_red_cube` -> `place_cube_in_box` -> `inspect_rgbd`

**Verified result** - **PASS**. The cube settles inside the open box, contacts the bottom, and moves `130.52 px` in the camera image.

<p align="center">
  <a href="showcase/01-button-cube-box/output/screenshots/dense_sequence.gif">
    <img src="showcase/01-button-cube-box/output/screenshots/dense_sequence.gif" alt="Button, cube, and box dense RGB interaction sequence" width="640">
  </a>
</p>

**Dense capture** - RGB frames are sampled every `0.20 s` of MuJoCo simulation time. The GIF uses a `200 ms` delay per frame (final frame `800 ms`); depth remains available for the verified keyframes.

[Environment](showcase/01-button-cube-box) | [Test report](showcase/01-button-cube-box/TEST_REPORT.md) | [Render report](showcase/01-button-cube-box/output/render_results.json) | [Dense report](showcase/01-button-cube-box/output/dense_sequence_results.json) | [Dense GIF](showcase/01-button-cube-box/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/01-button-cube-box/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** table, actuated button, free rigid cube, five-sided open box, and fixed RGB-D camera.
- **Validation:** scene spec, MJCF compilation, typed targets, dependency rejection, physics, reset, RGB-D, and task success.
- **Contract:** [interaction_manifest.json](showcase/01-button-cube-box/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/01-button-cube-box/output/sequence_results.json) | [GIF](showcase/01-button-cube-box/output/screenshots/sequence.gif) | [TIFF](showcase/01-button-cube-box/output/screenshots/sequence.tif)

</details>

---

### 02 / Smart Tool Cabinet

> **Prompt**
>
> Generate a desktop tool cabinet. Press the green unlock button, pull the drawer open by 22 cm, and verify the open state with a fixed camera. Keep the unlock point, handle, and camera checkpoint visible.

**Interaction** - `press_unlock_button` -> `pull_drawer_22cm` -> `inspect_open_drawer`

**Verified result** - **PASS**. The drawer reaches `0.218023 m` against a `0.22 m` target, the handle moves `54.21 px`, and depth changes across `13,700` pixels.

<p align="center">
  <a href="showcase/02-smart-drawer/output/screenshots/dense_sequence.gif">
    <img src="showcase/02-smart-drawer/output/screenshots/dense_sequence.gif" alt="Smart tool cabinet dense RGB interaction sequence" width="640">
  </a>
</p>

**Dense capture** - RGB frames are sampled every `0.20 s` of MuJoCo simulation time. The GIF uses a `200 ms` delay per frame (final frame `800 ms`); depth remains available for the verified keyframes.

[Environment](showcase/02-smart-drawer) | [Render report](showcase/02-smart-drawer/output/render_results.json) | [Dense report](showcase/02-smart-drawer/output/dense_sequence_results.json) | [Dense GIF](showcase/02-smart-drawer/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/02-smart-drawer/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** desktop cabinet, unlock button, slide-joint drawer, three visible markers, and fixed RGB-D camera.
- **Validation:** actuators, marker visibility, action dependencies, reset, RGB-D, drawer travel, and task success.
- **Contract:** [interaction_manifest.json](showcase/02-smart-drawer/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/02-smart-drawer/output/sequence_results.json) | [GIF](showcase/02-smart-drawer/output/screenshots/sequence.gif) | [TIFF](showcase/02-smart-drawer/output/screenshots/sequence.tif)

</details>

---

### 03 / Warehouse Navigation

> **Prompt**
>
> Generate a small warehouse where an orange robot reaches checkpoint A, navigates around the shelves to checkpoint B, and confirms completion with a top-view camera.

**Interaction** - `reach_checkpoint_a` -> `reach_checkpoint_b` -> `inspect_top_camera`

**Verified result** - **PASS**. The robot follows the south-side bypass with zero shelf contacts, keeps `0.270 m` minimum clearance, and moves `331.26 px` in the image.

<p align="center">
  <a href="showcase/03-warehouse-navigation/output/screenshots/dense_sequence.gif">
    <img src="showcase/03-warehouse-navigation/output/screenshots/dense_sequence.gif" alt="Warehouse navigation dense RGB interaction sequence" width="640">
  </a>
</p>

**Dense capture** - RGB frames are sampled every `0.20 s` of MuJoCo simulation time. The GIF uses a `200 ms` delay per frame (final frame `800 ms`); depth remains available for the verified keyframes.

[Environment](showcase/03-warehouse-navigation) | [Render report](showcase/03-warehouse-navigation/output/render_results.json) | [Dense report](showcase/03-warehouse-navigation/output/dense_sequence_results.json) | [Dense GIF](showcase/03-warehouse-navigation/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/03-warehouse-navigation/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** warehouse floor, collidable shelves, planar mobile robot, two checkpoints, and top-view RGB-D camera.
- **Validation:** route dependencies, clearance, zero shelf contact, marker visibility, RGB-D, reset, and task success.
- **Contract:** [interaction_manifest.json](showcase/03-warehouse-navigation/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/03-warehouse-navigation/output/sequence_results.json) | [GIF](showcase/03-warehouse-navigation/output/screenshots/sequence.gif) | [TIFF](showcase/03-warehouse-navigation/output/screenshots/sequence.tif)

</details>

---

### 04 / Lever and Ramp Ball

> **Prompt**
>
> Generate a workbench where a blue lever opens a gate, releases a purple ball down a ramp into a target tray, and verifies the result with a camera.

**Interaction** - `pull_blue_lever` -> `check_release_zone` -> `confirm_target_tray` -> `inspect_with_camera`

**Verified result** - **PASS**. The gate lifts `0.11656 m`; the ball enters the target tray and settles at `0.000277 m/s`; all four marker color families remain visible.

<p align="center">
  <a href="showcase/04-lever-ball-ramp/output/screenshots/dense_sequence.gif">
    <img src="showcase/04-lever-ball-ramp/output/screenshots/dense_sequence.gif" alt="Lever and ramp ball dense RGB interaction sequence" width="640">
  </a>
</p>

**Dense capture** - RGB frames are sampled every `0.20 s` of MuJoCo simulation time. The GIF uses a `200 ms` delay per frame (final frame `800 ms`); depth remains available for the verified keyframes.

[Environment](showcase/04-lever-ball-ramp) | [Render report](showcase/04-lever-ball-ramp/render_results.json) | [Dense report](showcase/04-lever-ball-ramp/output/dense_sequence_results.json) | [Physics report](showcase/04-lever-ball-ramp/physics_results.json) | [Dense GIF](showcase/04-lever-ball-ramp/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/04-lever-ball-ramp/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** workbench, actuated lever and gate, guarded ramp, free ball, open target tray, and fixed RGB-D camera.
- **Validation:** typed targets, four marker sites, dependency rejection, physical release and settling, RGB-D, reset, and task success.
- **Contract:** [interaction_manifest.json](showcase/04-lever-ball-ramp/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/04-lever-ball-ramp/output/sequence_results.json) | [GIF](showcase/04-lever-ball-ramp/output/screenshots/sequence.gif) | [TIFF](showcase/04-lever-ball-ramp/output/screenshots/sequence.tif)

</details>

---

## Usage

### Codex

Install or expose [`text2mujoco_codex`](text2mujoco_codex) as a skill, then describe the environment directly or invoke it explicitly:

```text
$text2mujoco
Generate a desktop tool cabinet, unlock it, pull the drawer open by 22 cm, and inspect it with a camera.
```

### Claude Code

Install [`text2mujoco_claude`](text2mujoco_claude) at `~/.claude/skills/text2mujoco/` or `.claude/skills/text2mujoco/`, then use a natural-language request or invoke:

```text
/text2mujoco Generate a warehouse navigation task with visible checkpoints and a top-view camera.
```

See the [Claude Code installation guide](text2mujoco_claude/README.md).

### Run a Generated Environment

Requirements: Python `>=3.9`, MuJoCo `3.2.7`, NumPy, and Pillow.

```bash
python -m pip install "mujoco==3.2.7" numpy pillow
cd showcase/02-smart-drawer
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

On macOS, `mjpython` gives the renderer access to the native CGL session. On headless Linux, try `MUJOCO_GL=egl` and then `MUJOCO_GL=osmesa` in separate processes.

Regenerate the original paced keyframe storyboards from the repository root:

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --scene all
```

Generate dense RGB captures sampled every `0.20 s` of simulation time:

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene all
```

The default GIFs are readable animations of discrete, physically verified interaction keyframes: each frame stays visible for `1.6 s`, and the final state stays for `2.6 s`. Dense GIFs preserve the first post-step state at each `0.20 s` simulation boundary and add action-boundary event frames; ordinary frames play at `200 ms` and the final frame at `800 ms`. Pass `--dense-interval <seconds>` to change the sampling interval. Dense GIFs contain RGB only, while the verified keyframe sequence retains RGB-D arrays. Both TIFF formats are full-resolution archives; TIFF playback timing is viewer-dependent.

<details>
<summary>Generated package structure and interaction API</summary>

```text
scene_spec.json             # Normalized natural-language request
model.xml                   # Loadable MuJoCo MJCF
environment.py              # Interaction, observation, reset, and success logic
interaction_manifest.json   # Targets, dependencies, and action schemas
physics_smoke.py            # Physics and state-machine validation
render_smoke.py             # RGB-D and visual-change validation
output/                     # Reports, screenshots, depth arrays, keyframe and dense GIF/TIFF sequences
```

```python
list_interaction_points()
get_action_schema()
reset(seed=None)
step({"id": "<interaction_id>", "payload": {}})
observe()
is_success()
```

</details>
