# 智能工具柜 (MuJoCo 3.2.7)

[English](README.md) · [中文](README.zh-CN.md)

这个生成的软件包实现了一个三步任务：按下绿色按钮解锁桌面工具柜，将抽屉向前拉出 `0.22 m`，然后从固定的检查相机采集 RGB-D 证据。

## 场景与交互

- `press_unlock_button` 将 `unlock_button_slide` 驱动到 `0.012 m`。一个黄色站点标记解锁点。
- `pull_drawer_22cm` 仅在解锁之后可用。一个位置执行器将 `drawer_slide` 驱动至 `0.22 m`；一个青色站点标记把手。
- `inspect_open_drawer` 要求抽屉位置至少达到 `0.218 m`。它从 `fixed_inspection_camera` 采集 `640 x 480` 的 RGB-D；一个洋红色站点标记检查点。

抽屉使用真实的滑动关节、位置执行器与多个碰撞 geom。拉动属于任务级控制指令；机器人夹爪的接触仿真在本场景范围之外。场景仅使用 MJCF 基本图元，无需下载任何网格资源。

环境暴露 `list_interaction_points()`、`get_action_schema()`、`reset(seed=None)`（返回初始观测）、`step({"id": "<interaction_id>", "payload": {}})`、`observe()` 与 `is_success()`。

## 文件

- `scene_spec.json`：归一化的场景、假设、传感器与动作契约。
- `model.xml`：可直接加载的 MJCF。
- `environment.py`：不含全局仿真器状态的环境 API。
- `interaction_manifest.json`：权威的机器可读交互契约与标记映射。
- `physics_smoke.py`：编译、动力学、拒绝分支、抽屉行程、重置以及 XML/MJB 重新加载检查。
- `render_smoke.py`：固定相机 RGB-D、标记可见性与把手像素位移检查。
- `output/sequence_results.json`：完整的故事板采集报告。
- `output/dense_sequence_results.json`：带 GIF/TIFF 帧映射的密集仿真时间采集报告。

## 运行

在本目录下：

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

在 macOS 上，`MUJOCO_GL=glfw` 使用 MuJoCo 原生的 CGL 上下文，`mjpython` 把进程连接到图形会话。在 Linux 上，请在各自独立的进程中依次尝试 `MUJOCO_GL=egl` 与 `MUJOCO_GL=osmesa`。

项目级收集器会在每次交互后写出一张 RGB-D 关键帧、一份有节奏的 `sequence.gif`、一张拼版 PNG 以及一份多页 TIFF 归档：

```bash
MUJOCO_GL=glfw mjpython ../../showcase/capture_sequences.py --scene 02-smart-drawer
```

该 GIF 让每个关键帧停留 `1.6 s`，最终状态停留 `2.6 s`。它以可读的方式动画呈现离散的交互关键帧，粒度停留在关键帧层面。TIFF 以全分辨率保存同一批关键帧，播放速度由各查看器自行决定。

如需从仿真时间采样得到更密集的 RGB 动画，请运行：

```bash
MUJOCO_GL=glfw mjpython ../../showcase/capture_sequences.py --dense --scene 02-smart-drawer
```

密集序列默认在每个 `0.20 s` 边界采样步进后的第一个状态；传入 `--dense-interval <seconds>` 可修改。它会额外加入动作边界的事件帧。`output/screenshots/dense_sequence.gif` 对普通帧使用 `200 ms` 延迟，最后一帧使用 `800 ms`，而 `output/screenshots/dense_sequence.tif` 保留全分辨率的 RGB 页面。报告记录时间戳与稳定的 `archive_frame` 索引；深度数据保留在经过验证的关键帧采集中。

## 证据

静态、物理与渲染报告分别保存为 `output/scene_spec_validation.json`、`output/physics_results.json` 与 `output/render_results.json`。只有 `status: PASS` 的渲染报告才算作证据，用以证明相机、可见标记、抽屉运动与任务成功确实经过验证。密集报告还会额外验证归档帧数、时序与非空白的 RGB 帧。
