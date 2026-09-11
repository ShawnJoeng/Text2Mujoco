# 面向 Claude Code 的 Text2MuJoCo

[English](README.md) · [中文](README.zh-CN.md)

这个目录是一个 Claude Code Agent Skill。[`SKILL.md`](SKILL.md) 是可被发现的 skill 入口；`references/` 和 `scripts/` 提供场景契约、MuJoCo 运行时指引、示例、校验清单和确定性校验器。

## 安装

为当前用户安装：

```bash
mkdir -p ~/.claude/skills
cp -R text2mujoco_claude ~/.claude/skills/text2mujoco
```

或者只为某一个项目安装：

```bash
mkdir -p .claude/skills
cp -R /path/to/Text2Mujoco/text2mujoco_claude .claude/skills/text2mujoco
```

安装完成后请开启一个新的 Claude Code 会话。这个 skill 可以由相关的自然语言请求自动选中，也可以显式调用：

```text
/text2mujoco Generate a desktop tool cabinet, unlock it, pull the drawer open by 22 cm, and inspect it with a camera.
```

仓库中的目录命名为 `text2mujoco_claude`，用来标识 Claude Code 适配器。安装后的目录使用 skill 名 `text2mujoco`，与 `SKILL.md` 中的 `name` 字段以及 `/text2mujoco` 命令保持一致。

## 校验

可以独立于 Claude Code 运行随包提供的场景规范校验器：

```bash
python3 scripts/validate_scene_spec.py /path/to/scene_spec.json --json
```

用 `python3 scripts/test_validate_scene_spec.py` 运行校验器回归测试套件。

`SKILL.md` 是真正的 skill 入口；外层的 GitHub 文件夹名只是一个分发标签。生成的报告使用包内相对路径，并且不包含本地主机名、凭据、用户路径或原始 traceback。
