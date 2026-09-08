# Text2MuJoCo for Codex

This directory is the Codex adapter for the Text2MuJoCo environment-generation workflow. The discoverable entrypoint is [`SKILL.md`](SKILL.md); `agents/openai.yaml` supplies the Codex UI metadata, while `references/` and `scripts/` contain the scene contract and validator.

## Install

Expose this directory as a Codex skill in the skills directory used by your Codex installation. Keep the following files together:

```text
text2mujoco_codex/
├── SKILL.md
├── agents/openai.yaml
├── references/
└── scripts/validate_scene_spec.py
```

The repository folder uses the explicit distribution name `text2mujoco_codex`. The skill front matter intentionally keeps the invocation name `text2mujoco`, so the explicit Codex command is:

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

Run the validator directly when checking a generated scene:

```bash
python3 scripts/validate_scene_spec.py /path/to/scene_spec.json --json
```

For physics-only checks use `MUJOCO_GL=disable`. On macOS use MuJoCo's `mjpython` with `MUJOCO_GL=glfw` for native CGL rendering; on Linux try EGL and OSMesa in separate processes.

The repository showcase collector writes paced GIF previews from discrete verified keyframes, plus full-resolution TIFF keyframe archives and RGB-D arrays. GIF playback uses an explicit delay; TIFF playback timing is viewer-dependent.
