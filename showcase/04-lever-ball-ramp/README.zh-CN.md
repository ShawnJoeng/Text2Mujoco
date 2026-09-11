# 杠杆与斜坡小球工作台

[English](README.md) · [中文](README.zh-CN.md)

这个生成的 MuJoCo 3.2.7 场景建模了一个工作台：蓝色杠杆抬起琥珀色闸门，紫色小球沿带护栏的斜坡滚入绿色目标托盘，一台固定的 RGB-D 相机验证结果。全部四个交互点都有可见的 MJCF 站点标记，并在 `interaction_manifest.json` 中以带类型的目标、依赖、效果与重置状态声明。

## 交互序列

`pull_blue_lever` -> `check_release_zone` -> `confirm_target_tray` -> `inspect_with_camera`

杠杆与闸门使用带位置执行器的真实铰链/滑动关节。小球使用自由关节；释放之后，MuJoCo 的重力、碰撞、摩擦、滚动与静置沉降决定它的运动。释放区与目标托盘的动作会推进物理并检查位置与速度。`inspect_with_camera` 会持久化 RGB 与深度观测。`reset(seed=None)` 返回初始观测，动作使用精确的 `{"id": ..., "payload": ...}` 信封结构。

## 文件

- `scene_spec.json`：归一化的场景契约与假设。
- `model.xml`：可编辑的 MJCF 源文件。
- `environment.py`：可执行的交互、观测、重置与成功判定 API。
- `interaction_manifest.json`：权威的带类型目标、标记站点、动作 schema 与依赖。
- `output/scene_spec_validation.json`：已提交的场景规范校验结果。
- `output/physics_results.json`：物理与交互校验结果。
- `output/render_results.json`：RGB-D 渲染与故事板校验结果。
- `output/model.xml`：按声明的输出设置产生的模型副本。
- `physics_smoke.py`：执行器、释放、静置沉降、接触、重置与重新加载检查。
- `render_smoke.py`：按后端选择的 RGB-D、标记可见性、前后变化与故事板检查。
- `output/sequence_results.json`：完整的关键帧采集报告。
- `output/dense_sequence_results.json`：带 GIF/TIFF 帧映射的密集仿真时间采集报告。

## 运行

在本目录下：

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

在 macOS 上，请使用 MuJoCo 的 `mjpython` 跳板并搭配 `MUJOCO_GL=glfw`，以便原生的 CGL 上下文可用。在 Linux 上，请在各自独立的进程中依次尝试 `MUJOCO_GL=egl` 与 `MUJOCO_GL=osmesa`。场景规范把后端保持为 `auto`；实际选用的后端来自进程环境。项目级收集器可以从仓库根目录重新生成完整的故事板：

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --scene 04-lever-ball-ramp
```

收集器会写出一份有节奏的 `sequence.gif`、一张拼版 PNG 以及一份多页 TIFF。该 GIF 让每个关键帧停留 `1.6 s`，最终状态停留 `2.6 s`；它以可读的方式动画呈现离散的交互关键帧，粒度停留在关键帧层面。TIFF 是全分辨率的关键帧归档，播放速度由各查看器自行决定。

如需从仿真时间采样得到更密集的 RGB 动画，请运行：

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene 04-lever-ball-ramp
```

密集序列默认在每个 `0.20 s` 边界采样步进后的第一个状态；传入 `--dense-interval <seconds>` 可修改。它会额外加入动作边界的事件帧。`output/screenshots/dense_sequence.gif` 对普通帧使用 `200 ms` 延迟，最后一帧使用 `800 ms`，而 `output/screenshots/dense_sequence.tif` 保留全分辨率的 RGB 页面。报告记录时间戳与稳定的 `archive_frame` 索引；深度数据保留在经过验证的关键帧采集中。

## 输出

场景写出 `output/physics_results.json`、`output/render_results.json`、`output/sequence_results.json` 与 `output/dense_sequence_results.json`，以及 `output/screenshots/` 下的关键帧与密集 GIF/TIFF 归档。分阶段的 RGB-D 数组保存在 `output/screenshots/sequence/` 下。`scene_spec.json` 是权威输入，`model.xml` 是可编辑的 MJCF 源文件。
