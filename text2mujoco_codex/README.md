# Text2MuJoCo for Codex

[English](README.md) · [中文](README.zh-CN.md)

This directory is the Codex adapter for the Text2MuJoCo environment-generation workflow. The discoverable entrypoint is [`SKILL.md`](SKILL.md); `agents/openai.yaml` supplies the Codex UI metadata, while `references/` and `scripts/` contain the scene contract and validator.

## Install

Install the adapter under the skill name `text2mujoco` and keep the following files together:

```text
text2mujoco_codex/
├── SKILL.md
├── agents/openai.yaml
├── references/
├── scripts/validate_scene_spec.py
└── scripts/test_validate_scene_spec.py
```

The repository folder is named `text2mujoco_codex` to identify the Codex adapter. With the Codex skill installer, use `--name text2mujoco` so the installed skill and explicit invocation name match:

```bash
python3 /path/to/skill-installer/scripts/install-skill-from-github.py \
  --repo ShawnJoeng/Text2Mujoco \
  --path text2mujoco_codex \
  --name text2mujoco
```

After installation, invoke the skill with:

```text
$text2mujoco
Generate a desktop tool cabinet, unlock it, pull the drawer open by 22 cm, and inspect it with a camera.
```

Natural-language requests that describe a MuJoCo scene or task can also trigger the skill automatically.

## Workflow

The skill converts the request into:

- `scene_spec.json`, a normalized, validated scene contract;
- `model.xml`, loadable MuJoCo MJCF;
- `environment.py`, executable interaction and observation logic;
- `interaction_manifest.json`, typed targets, dependencies, and action schemas;
- physics and render smoke tests with machine-readable reports.

The manifest is the canonical package-root-relative interaction contract; the runtime checks it against `scene_spec.json` before executing actions. Capture and artifact output paths must stay inside the generated package so reports remain portable.

Run the validator directly when checking a generated scene:

```bash
python3 scripts/validate_scene_spec.py /path/to/scene_spec.json --json
```

Run the validator regression suite with `python3 scripts/test_validate_scene_spec.py`.

For physics-only checks use `MUJOCO_GL=disable`. On macOS use MuJoCo's `mjpython` with `MUJOCO_GL=glfw` for native CGL rendering and an active graphics session; on Linux try EGL and OSMesa in separate processes.

The repository showcase collector writes paced GIF previews from discrete verified keyframes and dense RGB captures sampled in simulation time. It also writes full-resolution TIFF archives and RGB-D arrays. GIF playback uses an explicit delay; TIFF playback timing is viewer-dependent. See the [project showcase](../README.md#showcase) for the generated examples.
