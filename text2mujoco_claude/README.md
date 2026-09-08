# Text2MuJoCo for Claude Code

This directory is a Claude Code Agent Skill. [`SKILL.md`](SKILL.md) is the discoverable skill entrypoint; `references/` and `scripts/` provide the scene contract, MuJoCo runtime guidance, examples, validation checklist, and deterministic validator.

## Install

Install it for the current user:

```bash
mkdir -p ~/.claude/skills
cp -R text2mujoco_claude ~/.claude/skills/text2mujoco
```

Or install it for one project:

```bash
mkdir -p .claude/skills
cp -R /path/to/Text2Mujoco/text2mujoco_claude .claude/skills/text2mujoco
```

Start a new Claude Code session after installation. The skill can be selected automatically from a relevant natural-language request, or invoked explicitly:

```text
/text2mujoco Generate a desktop tool cabinet, unlock it, pull the drawer open by 22 cm, and inspect it with a camera.
```

The repository directory is named `text2mujoco_claude` to identify the Claude Code adapter. The installed directory uses the skill name `text2mujoco`, matching the `name` field in `SKILL.md` and the `/text2mujoco` command.

## Validate

Run the bundled scene-spec validator independently of Claude Code:

```bash
python3 scripts/validate_scene_spec.py /path/to/scene_spec.json --json
```

Run the validator regression suite with `python3 scripts/test_validate_scene_spec.py`.

`SKILL.md` is the actual skill entrypoint; the outer GitHub folder name is only a distribution label. Generated reports use package-relative paths and do not include local hostnames, credentials, user paths, or raw tracebacks.
