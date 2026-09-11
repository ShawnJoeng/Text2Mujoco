# 机器人定位销装配工作单元

[English](README.md) · [中文](README.zh-CN.md)

这个 MuJoCo 3.2.7 展示场景是一个紧凑的机器人工作单元：一条平面橙色机械臂把夹爪下降到一枚红色定位销上，把它搬运到蓝色夹具处，插入，释放，并用一台固定的 RGB-D 相机核验就位结果。

## 交互序列

1. `move_arm_to_peg` 把三个铰链关节移动到定位销的接近位姿。
2. `grasp_peg_with_arm` 闭合被驱动的夹爪，并按夹爪合拢时测得的偏移启用 `peg_grasp` 焊接约束。
3. `move_arm_to_socket` 用机械臂执行器搬运被抓住的定位销。
4. `insert_peg_into_socket` 驱动工具升降滑动关节向下进入导向夹具。
5. `release_assembled_peg` 打开夹爪，让重力/接触把定位销沉降到插孔中。
6. `inspect_assembly` 从 `assembly_camera` 采集 RGB-D 证据。

所有交互点都有具名且不发生碰撞的标记站点。环境暴露 `list_interaction_points()`、`get_action_schema()`、`reset(seed=None)`、`step({"id": "<interaction_id>", "payload": {}})`、`observe()` 和 `is_success()`。

## 运行

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

在 Linux 上尝试独立渲染时，请使用 `MUJOCO_GL=egl` 或 `MUJOCO_GL=osmesa`。RGB-D 采集结果写入该包之下；产物路径限定在包范围内，并拒绝符号链接穿越。

证据：[`dense_sequence.gif`](output/screenshots/dense_sequence.gif) | [`dense_sequence.tif`](output/screenshots/dense_sequence.tif) | [`dense_sequence_results.json`](output/dense_sequence_results.json)。常规 RGB 帧按仿真时间每 `0.20 s` 采样一次；GIF 每帧使用 `200 ms`，最后一帧使用 `800 ms`。

## 文件

- `scene_spec.json`：归一化场景、机器人导出、传感器与交互契约。
- `model.xml`：自包含的 MJCF，包含显式的机械臂、工具、夹爪、定位销、夹具、关节与执行器。
- `environment.py`：确定性运行时 API 与安全的产物持久化。
- `interaction_manifest.json`：标准的机器可读依赖顺序与标记映射。
- `physics_smoke.py`：MJCF、关节、执行器、依赖、搬运、释放与 reset 检查。
- `render_smoke.py`：RGB-D 有效性、标记可见性、机械臂运动与 reset 检查。
