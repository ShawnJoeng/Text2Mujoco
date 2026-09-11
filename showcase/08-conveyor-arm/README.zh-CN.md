# 输送带到机械臂交接工作单元

[English](README.md) · [中文](README.zh-CN.md)

这个 MuJoCo 3.2.7 展示场景演示一套输送带与机器人协同的工作流。一个带驱动的滚筒把蓝色包裹送到取件点，一条关节化的橙色机械臂用平行夹爪夹住包裹，把它运到绿色目标料箱，释放，并用一台固定的 RGB-D 相机核验交接结果。红色料箱作为可见的干扰物保留在场景中。

## 交互序列

1. `start_conveyor_to_pickup` 驱动输送带滚筒，包裹随之前进到取件索引位置。
2. `move_arm_to_parcel` 移动机械臂关节与工具升降，进入交接位姿。
3. `grasp_parcel_with_arm` 闭合两个带驱动的手指滑动关节，并按夹爪合拢时测得的偏移启用 `parcel_grasp` 焊接约束。
4. `move_arm_to_target_bin` 把握住的包裹抬到料箱边沿上方，以笛卡尔直线子步横移，然后用机械臂执行器把它放低进入料箱。
5. `release_parcel_in_target_bin` 打开两个手指，关闭保持，并让包裹在五片式目标料箱中沉降。
6. `inspect_handoff` 从 `handoff_camera` 采集 RGB-D 证据。

环境暴露 `list_interaction_points()`、`get_action_schema()`、`reset(seed=None)`、`step({"id": "<interaction_id>", "payload": {}})`、`observe()` 和 `is_success()`。每个交互点都有一个可见的具名标记站点。

## 运行

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

在独立的 Linux 进程中使用 `MUJOCO_GL=egl` 或 `MUJOCO_GL=osmesa`。采集输出与保存的模型产物都保持相对于该包，并拒绝符号链接穿越。

证据：[`dense_sequence.gif`](output/screenshots/dense_sequence.gif) | [`dense_sequence.tif`](output/screenshots/dense_sequence.tif) | [`dense_sequence_results.json`](output/dense_sequence_results.json)。常规 RGB 帧按仿真时间每 `0.20 s` 采样一次；GIF 每帧使用 `200 ms`，最后一帧使用 `800 ms`。

## 文件

- `scene_spec.json`：输送带、机器人、包裹、料箱、传感器与交互契约。
- `model.xml`：自包含的 MJCF，包含带驱动的输送带铰链、关节化机械臂、双夹爪滑动关节、自由包裹、料箱与标记。
- `environment.py`：输送带/机械臂协同的运行时 API 与安全的产物持久化。
- `interaction_manifest.json`：标准的机器可读依赖顺序与标记映射。
- `physics_smoke.py`：输送带送件、关节/执行器、抓取搬运、释放与 reset 检查。
- `render_smoke.py`：RGB-D 有效性、标记可见性、机械臂运动与 reset 检查。
