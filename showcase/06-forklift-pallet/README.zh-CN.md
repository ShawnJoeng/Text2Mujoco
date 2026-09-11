# 叉车托盘配送

[English](README.md) · [中文](README.zh-CN.md)

这个 MuJoCo 3.2.7 展示场景演示一台移动叉车绕过货架把装载好的托盘送到指定位置。橙色叉车开到托盘处，抬起带驱动的货叉，接合自由托盘刚体，沿标记好的路线搬运，把货叉降到绿色配送区，并用一台固定的 RGB-D 相机核验结果。

## 交互序列

`drive_to_pallet` -> `raise_forks` -> `engage_pallet` -> `carry_to_drop_zone` -> `lower_forks_release` -> `inspect_forklift_delivery`

叉车拥有带驱动的平面底盘滑动关节、一个转向铰链和一个垂直的货叉升降滑动关节。一旦接合，托盘就由 `fork_carriage` 与托盘之间的 `pallet_grasp` 焊接约束携带，同时 MuJoCo 继续推进物理。释放动作会关闭该焊接约束、降下货叉，并让重力与配送区接触使托盘沉降到位。每个交互点都有一个可见的具名标记站点和一条显式依赖。

环境暴露 `list_interaction_points()`、`get_action_schema()`、`reset(seed=None)`、`step({"id": "<interaction_id>", "payload": {}})`、`observe()` 和 `is_success()`。

## 文件

- `scene_spec.json`：归一化的仓储请求、叉车/托盘资产、路线、传感器与交互契约。
- `model.xml`：自包含的 MJCF，包含移动叉车关节、带驱动的货叉、自由托盘、货架障碍、配送区、标记与相机。
- `environment.py`：动作校验、确定性路线控制器、托盘同步、RGB-D 采集、reset 与成功判定。
- `interaction_manifest.json`：标准的带类型目标、依赖顺序与标记映射。
- `physics_smoke.py`：模型、执行器、路线、托盘跟随、释放、reset 与产物重载检查。
- `render_smoke.py`：RGB-D 有效性、标记可见性、叉车运动与 reset 检查。

## 运行

在本目录下：

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

在 Linux 上，请在独立进程中使用 `MUJOCO_GL=egl` 或 `MUJOCO_GL=osmesa`。在仓库根目录下，用以下命令生成密集的 200 ms RGB GIF/TIFF：

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene 06-forklift-pallet --dense-interval 0.20
```

所有传感器与产物路径都相对于该包，并拒绝绝对路径、路径穿越、URI 与符号链接逃逸。

证据：[`dense_sequence.gif`](output/screenshots/dense_sequence.gif) | [`dense_sequence.tif`](output/screenshots/dense_sequence.tif) | [`dense_sequence_results.json`](output/dense_sequence_results.json)。常规 RGB 帧按仿真时间每 `0.20 s` 采样一次；GIF 每帧使用 `200 ms`，最后一帧使用 `800 ms`。
