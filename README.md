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

Text2MuJoCo is an agent skill package, not a standalone natural-language compiler. Used with Codex or Claude Code, it turns a scene or task description into a loadable MuJoCo package. It resolves objects, physics, sensors, action order, success conditions, and visible interaction points, then validates the result with machine-readable reports. A set of ready-to-run [sample queries](showcase/sample_queries.json) illustrates the input format.

Use the [Codex skill](text2mujoco_codex/README.md) or the [Claude Code skill](text2mujoco_claude/README.md).

## Showcase

### 01 / Button, Cube, and Box

> **Prompt**
>
>  Place a button, a red cube, and an open box on a table. Press the button, grasp the cube, place it in the box, and inspect the result with an RGB-D camera.

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
>  Generate a desktop tool cabinet. Press the green unlock button, pull the drawer open by 22 cm, and verify the open state with a fixed camera. Keep the unlock point, handle, and camera checkpoint visible.

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

[Environment](showcase/04-lever-ball-ramp) | [Render report](showcase/04-lever-ball-ramp/output/render_results.json) | [Dense report](showcase/04-lever-ball-ramp/output/dense_sequence_results.json) | [Physics report](showcase/04-lever-ball-ramp/output/physics_results.json) | [Dense GIF](showcase/04-lever-ball-ramp/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/04-lever-ball-ramp/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** workbench, actuated lever and gate, guarded ramp, free ball, open target tray, and fixed RGB-D camera.
- **Validation:** typed targets, four marker sites, dependency rejection, physical release and settling, RGB-D, reset, and task success.
- **Contract:** [interaction_manifest.json](showcase/04-lever-ball-ramp/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/04-lever-ball-ramp/output/sequence_results.json) | [GIF](showcase/04-lever-ball-ramp/output/screenshots/sequence.gif) | [TIFF](showcase/04-lever-ball-ramp/output/screenshots/sequence.tif)

</details>

---

### 05 / Robotic Arm Sorting Cell

> **Prompt**
>
> Create a tabletop robotic sorting cell. A three-joint arm approaches a blue part on a conveyor, closes its parallel gripper, transfers the part to a blue bin beside a red distractor bin, releases it, and verifies the result with a fixed RGB-D camera.

**Interaction** - `approach_blue_part` -> `grasp_blue_part` -> `transfer_to_blue_bin` -> `release_blue_part` -> `inspect_sorting_result`

**Verified result** - **PASS**. MuJoCo 3.2.7 physics confirms five articulated joints, five arm/gripper actuators, dependency enforcement, synchronized grasp transport, and release inside the blue bin. OSMesa rendering confirms `0.600 m` of blue-part motion, finite RGB-D, and all marker color families.

<p align="center">
  <a href="showcase/05-robot-arm-sorting/output/screenshots/dense_sequence.gif">
    <img src="showcase/05-robot-arm-sorting/output/screenshots/dense_sequence.gif" alt="Robotic arm sorting cell dense RGB interaction sequence" width="640">
  </a>
</p>

**Dense capture** - 29 RGB frames are sampled from MuJoCo simulation time over `4.768 s`; regular frames are exactly `0.20 s` apart, action-boundary frames are retained, GIF frames use `200 ms` (final `800 ms`), and the TIFF has 29 matching pages.

[Environment](showcase/05-robot-arm-sorting) | [Physics report](showcase/05-robot-arm-sorting/output/physics_results.json) | [Render report](showcase/05-robot-arm-sorting/output/render_results.json) | [Dense report](showcase/05-robot-arm-sorting/output/dense_sequence_results.json) | [Dense GIF](showcase/05-robot-arm-sorting/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/05-robot-arm-sorting/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** worktable, conveyor, three-link arm, dual-finger gripper, blue/red parts and bins, five visible interaction markers, and a fixed RGB-D camera.
- **Synchronization:** the blue free body follows `arm_tcp` only during the documented task-level grasp; release disables the hold before contact settling.
- **Contract:** [interaction_manifest.json](showcase/05-robot-arm-sorting/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/05-robot-arm-sorting/output/sequence_results.json) | [Storyboard](showcase/05-robot-arm-sorting/output/screenshots/sequence.png) | [TIFF](showcase/05-robot-arm-sorting/output/screenshots/sequence.tif)

</details>

---

### 06 / Forklift Pallet Delivery

> **Prompt**
>
> Create a warehouse forklift task. An orange mobile forklift drives to a loaded pallet, raises its powered forks, engages the pallet, carries it around a storage rack to a green delivery zone, lowers the forks to release the load, and verifies delivery with an RGB-D camera.

**Interaction** - `drive_to_pallet` -> `raise_forks` -> `engage_pallet` -> `carry_to_drop_zone` -> `lower_forks_release` -> `inspect_forklift_delivery`

**Verified result** - **PASS**. MuJoCo 3.2.7 physics confirms mobile base joints, powered fork lift, zero rack contacts, synchronized pallet transport, release settling, and deterministic reset. OSMesa rendering confirms `2.998 m` of forklift motion, finite RGB-D, and all five marker color families.

<p align="center">
  <a href="showcase/06-forklift-pallet/output/screenshots/dense_sequence.gif">
    <img src="showcase/06-forklift-pallet/output/screenshots/dense_sequence.gif" alt="Forklift pallet delivery dense RGB interaction sequence" width="640">
  </a>
</p>

**Dense capture** - 43 RGB frames are sampled from MuJoCo simulation time over `7.270 s`; regular frames are exactly `0.20 s` apart, action-boundary frames are retained, GIF frames use `200 ms` (final `800 ms`), and the TIFF has 43 matching pages.

[Environment](showcase/06-forklift-pallet) | [Physics report](showcase/06-forklift-pallet/output/physics_results.json) | [Render report](showcase/06-forklift-pallet/output/render_results.json) | [Dense report](showcase/06-forklift-pallet/output/dense_sequence_results.json) | [Dense GIF](showcase/06-forklift-pallet/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/06-forklift-pallet/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** loading and delivery pads, mobile forklift, raised fork carriage, loaded pallet and crate, storage rack obstacle, open delivery zone, six markers, and a fixed top-view RGB-D camera.
- **Synchronization:** the pallet follows the fork anchor while engaged; lowering the forks removes the hold and lets the pallet settle in the delivery zone.
- **Contract:** [interaction_manifest.json](showcase/06-forklift-pallet/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/06-forklift-pallet/output/sequence_results.json) | [Storyboard](showcase/06-forklift-pallet/output/screenshots/sequence.png) | [TIFF](showcase/06-forklift-pallet/output/screenshots/sequence.tif)

</details>

---

### 07 / Robot Peg Assembly

> **Prompt**
>
> Build a robot assembly station. An orange arm moves to a red locating peg, closes its gripper to pick it up, transports it to a blue fixture, inserts it vertically, releases it, and verifies the assembly with a fixed RGB-D camera.

**Interaction** - `move_arm_to_peg` -> `grasp_peg_with_arm` -> `move_arm_to_socket` -> `insert_peg_into_socket` -> `release_assembled_peg` -> `inspect_assembly`

**Verified result** - **PASS**. MuJoCo 3.2.7 physics confirms three arm hinges, tool lift, gripper slide, synchronized held-peg transport, finite state, and a `0.0708 m` socket release tolerance. OSMesa rendering confirms `0.378 m` of tool motion, finite RGB-D, and all marker color families.

<p align="center">
  <a href="showcase/07-robot-assembly/output/screenshots/dense_sequence.gif">
    <img src="showcase/07-robot-assembly/output/screenshots/dense_sequence.gif" alt="Robot peg assembly dense RGB interaction sequence" width="640">
  </a>
</p>

**Dense capture** - 28 RGB frames are sampled from MuJoCo simulation time over `4.320 s`; regular frames are exactly `0.20 s` apart, action-boundary frames are retained, GIF frames use `200 ms` (final `800 ms`), and the TIFF has 28 matching pages.

[Environment](showcase/07-robot-assembly) | [Physics report](showcase/07-robot-assembly/output/physics_results.json) | [Render report](showcase/07-robot-assembly/output/render_results.json) | [Dense report](showcase/07-robot-assembly/output/dense_sequence_results.json) | [Dense GIF](showcase/07-robot-assembly/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/07-robot-assembly/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** workbench, three-link arm, vertical tool lift, gripper, free red peg, blue insertion fixture, six visible markers, and a fixed RGB-D camera.
- **Synchronization:** the peg follows the tool during the documented grasp; insertion and release are separate dependency-checked actions.
- **Contract:** [interaction_manifest.json](showcase/07-robot-assembly/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/07-robot-assembly/output/sequence_results.json) | [Storyboard](showcase/07-robot-assembly/output/screenshots/sequence.png) | [TIFF](showcase/07-robot-assembly/output/screenshots/sequence.tif)

</details>

---

### 08 / Conveyor-to-Arm Handoff

> **Prompt**
>
> Create a synchronized conveyor-to-arm handoff cell. A conveyor moves a blue parcel to a pickup point, a three-joint arm closes its parallel gripper around it, carries it to a green target bin, releases it, and verifies the handoff with a fixed RGB-D camera.

**Interaction** - `start_conveyor_to_pickup` -> `move_arm_to_parcel` -> `grasp_parcel_with_arm` -> `move_arm_to_target_bin` -> `release_parcel_in_target_bin` -> `inspect_handoff`

**Verified result** - **PASS**. MuJoCo 3.2.7 physics confirms a powered conveyor hinge, three arm hinges, tool lift, dual gripper slides, parcel delivery, synchronized held transport, and target-bin settling. OSMesa rendering confirms `1.045 m` of tool motion, finite RGB-D, and all marker color families.

<p align="center">
  <a href="showcase/08-conveyor-arm/output/screenshots/dense_sequence.gif">
    <img src="showcase/08-conveyor-arm/output/screenshots/dense_sequence.gif" alt="Conveyor-to-arm handoff dense RGB interaction sequence" width="640">
  </a>
</p>

**Dense capture** - 30 RGB frames are sampled from MuJoCo simulation time over `4.672 s`; regular frames are exactly `0.20 s` apart, action-boundary frames are retained, GIF frames use `200 ms` (final `800 ms`), and the TIFF has 30 matching pages.

[Environment](showcase/08-conveyor-arm) | [Physics report](showcase/08-conveyor-arm/output/physics_results.json) | [Render report](showcase/08-conveyor-arm/output/render_results.json) | [Dense report](showcase/08-conveyor-arm/output/dense_sequence_results.json) | [Dense GIF](showcase/08-conveyor-arm/output/screenshots/dense_sequence.gif) | [Dense TIFF](showcase/08-conveyor-arm/output/screenshots/dense_sequence.tif)

<details>
<summary>Scene and validation details</summary>

- **Scene:** powered conveyor, three-link arm, vertical tool lift, dual-finger gripper, blue parcel, green/red bins, six visible markers, and a fixed RGB-D camera.
- **Synchronization:** the parcel advances under the conveyor controller, follows `arm_tcp` while grasped, and is released before physical settling.
- **Contract:** [interaction_manifest.json](showcase/08-conveyor-arm/interaction_manifest.json)
- **Keyframe archive:** [Sequence JSON](showcase/08-conveyor-arm/output/sequence_results.json) | [Storyboard](showcase/08-conveyor-arm/output/screenshots/sequence.png) | [TIFF](showcase/08-conveyor-arm/output/screenshots/sequence.tif)

</details>

---

## Usage

### Codex

Install the [`text2mujoco_codex`](text2mujoco_codex) adapter with the Codex skill installer. The `--name` value keeps the installed skill name consistent with the `SKILL.md` front matter:

```bash
python3 /path/to/skill-installer/scripts/install-skill-from-github.py \
  --repo ShawnJoeng/Text2Mujoco \
  --path text2mujoco_codex \
  --name text2mujoco
```

Then describe the environment directly or invoke it explicitly:

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

The collector writes only under the repository `showcase/` tree so sensor and artifact paths remain portable; its `--output-root` option accepts that tree only.

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
showcase/validate_manifests.py # Cross-scene manifest/spec parity validation
showcase/dense_archive_test.py # 0.20 s GIF/TIFF archive regression checks
output/                     # Reports, screenshots, depth arrays, keyframe and dense GIF/TIFF sequences
```

```python
list_interaction_points()
get_action_schema()
reset(seed=None)  # returns the initial observation
step({"id": "<interaction_id>", "payload": {}})
observe()
is_success()
```

</details>
