# 面向 Codex 的 Text2MuJoCo

[English](README.md) · [中文](README.zh-CN.md)

这个目录是 Text2MuJoCo 环境生成工作流的 Codex 适配器。可被发现的入口是 [`SKILL.md`](SKILL.md)；`agents/openai.yaml` 提供 Codex UI 元数据，`references/` 和 `scripts/` 则包含场景契约和校验器。

## 安装

以 skill 名 `text2mujoco` 安装这个适配器，并把下列文件放在一起：

```text
text2mujoco_codex/
├── SKILL.md
├── agents/openai.yaml
├── references/
├── scripts/validate_scene_spec.py
└── scripts/test_validate_scene_spec.py
```

仓库中的文件夹命名为 `text2mujoco_codex`，用来标识 Codex 适配器。使用 Codex skill 安装器时，加上 `--name text2mujoco`，让安装后的 skill 名与显式调用名保持一致：

```bash
python3 /path/to/skill-installer/scripts/install-skill-from-github.py \
  --repo ShawnJoeng/Text2Mujoco \
  --path text2mujoco_codex \
  --name text2mujoco
```

安装完成后，用下面的方式调用这个 skill：

```text
$text2mujoco
Generate a desktop tool cabinet, unlock it, pull the drawer open by 22 cm, and inspect it with a camera.
```

描述 MuJoCo 场景或任务的自然语言请求也可以自动触发这个 skill。

## 工作流

这个 skill 把请求转换成：

- `scene_spec.json`，一份归一化并已校验的场景契约；
- `model.xml`，可加载的 MuJoCo MJCF；
- `environment.py`，可执行的交互与观测逻辑；
- `interaction_manifest.json`，带类型的目标、依赖关系和动作 schema；
- 物理与渲染 smoke 测试，附带机器可读的报告。

这份清单是规范的、相对包根目录的交互契约；运行时会在执行动作之前把它与 `scene_spec.json` 对照检查。截图与产物的输出路径必须留在生成的包内部，这样报告才能保持可移植。

检查一个生成出来的场景时，可以直接运行校验器：

```bash
python3 scripts/validate_scene_spec.py /path/to/scene_spec.json --json
```

用 `python3 scripts/test_validate_scene_spec.py` 运行校验器回归测试套件。

只做物理检查时使用 `MUJOCO_GL=disable`。在 macOS 上使用 MuJoCo 的 `mjpython` 配合 `MUJOCO_GL=glfw`，以获得原生 CGL 渲染和一个活跃的图形会话；在 Linux 上则在各自独立的进程中分别尝试 EGL 和 OSMesa。

仓库的展示收集器会写出带节奏的 GIF 预览，素材来自离散的已验证关键帧和按仿真时间密集采样的 RGB 截图。它同时写出全分辨率的 TIFF 归档和 RGB-D 数组。GIF 播放使用显式指定的延迟；TIFF 的播放节奏取决于查看器。生成的示例见[项目展示](../README.md#showcase)。
