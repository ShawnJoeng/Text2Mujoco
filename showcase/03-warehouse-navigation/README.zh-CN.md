# 小型仓库导航 (MuJoCo 3.2.7)

[English](README.md) · [中文](README.zh-CN.md)

这个生成的软件包实现了一个有序的仓库导航任务。一台橙色平面机器人从西南角出发，到达黄色检查点 A，沿主货架南侧经过三个安全航点抵达绿色检查点 B，最后用固定的四分之三俯视相机采集 `640 x 480` 的 RGB-D 证据。

## 场景与交互

- `reach_checkpoint_a` 从起点驱动到 `checkpoint_a_marker`，最终位置误差不超过 `0.10 m`。
- `reach_checkpoint_b` 仅在 A 之后可用。控制器在抵达 B 之前依次经过 `[-1.25, -1.35]`、`[0.75, -1.35]` 与 `[1.15, -0.95]`。
- `inspect_top_camera` 仅在 B 之后可用，并从 `top_camera` 采集 RGB-D。

黄色、绿色与洋红色的无碰撞标记站点分别标出 A、B 与相机检查点。机器人使用两个正交的滑动关节与位置执行器实现确定性的平面导航。这是一个任务级运动控制器；差速驱动与轮胎滑移属于模型范围之外的细节。货架 geom 是可碰撞的 box 图元，控制器在运动与静置沉降过程中检查真实的 MuJoCo 接触。

环境暴露 `list_interaction_points()`、`get_action_schema()`、`reset(seed=None)`（返回初始观测）、`step({"id": "<interaction_id>", "payload": {}})`、`observe()` 与 `is_success()`。

## 文件

- `scene_spec.json`：归一化的场景、假设、传感器、动作依赖与任务条件。
- `model.xml`：可直接加载的 MJCF 场景。
- `environment.py`：环境 API、动作校验、依赖检查、航点控制、碰撞检查与 RGB-D 采集。
- `interaction_manifest.json`：权威的机器可读交互点、可见标记与动作 schema。
- `physics_smoke.py`：MJCF 编译、动力学、非法动作、路线安全、碰撞、重置以及 XML/MJB 重新加载校验。
- `render_smoke.py`：真实的俯视 RGB-D、标记可见性、机器人像素位移、任务成功与渲染重置校验。
- `output/sequence_results.json`：完整的故事板采集报告。
- `output/dense_sequence_results.json`：带 GIF/TIFF 帧映射的密集仿真时间采集报告。

## 运行

在本目录下：

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

在 macOS 上，`MUJOCO_GL=glfw` 使用 MuJoCo 原生的 CGL 上下文，并需要访问当前的图形会话。在 Linux 上，请在各自独立的进程中依次尝试 EGL 与 OSMesa。

项目级收集器会在每次交互后写出一张 RGB-D 关键帧、一份有节奏的 `sequence.gif`、一张拼版 PNG 以及一份多页 TIFF 归档：

```bash
MUJOCO_GL=glfw mjpython ../../showcase/capture_sequences.py --scene 03-warehouse-navigation
```

该 GIF 让每个关键帧停留 `1.6 s`，最终状态停留 `2.6 s`。它以可读的方式动画呈现离散的交互关键帧，粒度停留在关键帧层面。TIFF 以全分辨率保存同一批关键帧，播放速度由各查看器自行决定。

如需从仿真时间采样得到更密集的 RGB 动画，请运行：

```bash
MUJOCO_GL=glfw mjpython ../../showcase/capture_sequences.py --dense --scene 03-warehouse-navigation
```

密集序列默认在每个 `0.20 s` 边界采样步进后的第一个状态；传入 `--dense-interval <seconds>` 可修改。它会额外加入动作边界的事件帧。`output/screenshots/dense_sequence.gif` 对普通帧使用 `200 ms` 延迟，最后一帧使用 `800 ms`，而 `output/screenshots/dense_sequence.tif` 保留全分辨率的 RGB 页面。报告记录时间戳与稳定的 `archive_frame` 索引；深度数据保留在经过验证的关键帧采集中。

## 证据

静态、物理与渲染报告分别保存为 `output/scene_spec_validation.json`、`output/physics_results.json` 与 `output/render_results.json`。只有 `status: PASS` 的渲染报告才算作证据，用以证明截图来自 MuJoCo、三个标记全部可见、机器人安全移动、任务完成，并且重置恢复了初始状态。密集报告还会额外验证归档帧数、时序与非空白的 RGB 帧。
