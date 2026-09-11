# 机械臂分拣工作单元

[English](README.md) · [中文](README.zh-CN.md)

这个 MuJoCo 展示场景是一个紧凑的工业取放工作单元。一条带驱动肩、肘、腕的机械臂靠近输送带上的蓝色圆柱零件，闭合平行夹爪，把零件搬运到蓝色料箱上方，松开零件，并用一台固定的 RGB-D 相机核验分拣结果。一个红色零件和一个红色料箱提供了可见的干扰通道。

## 交互序列

`approach_blue_part` -> `grasp_blue_part` -> `transfer_to_blue_bin` -> `release_blue_part` -> `inspect_sorting_result`

前四个动作是可执行的机器人控制。机械臂使用三个铰链关节和位置执行器；两个夹爪手指使用带驱动的滑动关节。在抓取阶段，蓝色零件由 `tool_turret` 与该零件之间的 `blue_part_grasp` 焊接约束携带，其偏移在夹爪合拢时测得，同时 MuJoCo 继续推进物理。释放动作把零件放到打开的蓝色料箱上方，并在检查其局部内部边界之前运行沉降步。`inspect_sorting_result` 需要启用 MuJoCo 渲染后端，并把 RGB 与深度数组写入该包目录下。

公开 API 为 `list_interaction_points()`、`get_action_schema()`、`reset(seed=None)`、`step({"id": "<interaction_id>", "payload": {}})`、`observe()` 和 `is_success()`。

## 文件

- `scene_spec.json`：归一化场景、机器人资产、交互依赖、传感器与输出契约。
- `model.xml`：可直接加载的 MJCF，包含关节化机械臂、带驱动的夹爪、零件、料箱、桌面、输送带、标记与相机。
- `environment.py`：动作校验、确定性 IK 路点、夹爪控制、抓取同步、RGB-D 采集、reset 与成功判定。
- `interaction_manifest.json`：标准的带类型交互契约与依赖顺序。
- `physics_smoke.py`：模型名称、执行器、IK、抓取/搬运/释放、非法动作、reset 与产物重载检查。
- `render_smoke.py`：真实相机 RGB-D、标记可见性、物体位移与最终状态检查。

## 运行

在本目录下：

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

在 Linux 上，请在独立进程中使用 `MUJOCO_GL=egl` 或 `MUJOCO_GL=osmesa`。在仓库根目录下，这个场景注册之后，项目的采集收集器就能生成节奏化的关键帧归档与密集 RGB 归档：

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --scene 05-robot-arm-sorting
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene 05-robot-arm-sorting --dense-interval 0.20
```

密集归档每 0.20 秒采样一次仿真时间，并把同样的 RGB 帧存入多页 TIFF；收集器的报告会记录帧时间戳与归档索引。

证据：[`dense_sequence.gif`](output/screenshots/dense_sequence.gif) | [`dense_sequence.tif`](output/screenshots/dense_sequence.tif) | [`dense_sequence_results.json`](output/dense_sequence_results.json)。
