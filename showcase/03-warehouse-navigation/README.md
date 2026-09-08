# 小型仓库导航（MuJoCo 3.2.7）

此包实现一个按依赖顺序执行的仓库导航任务：橙色移动机器人从西南角起点出发，先到达黄色检查点 A，再经主货架南侧的三个安全航点绕行到绿色检查点 B，最后用固定顶视相机采集 `640 x 480` RGB-D 证据。

## 场景与交互

- `reach_checkpoint_a`：从起点导航至 `checkpoint_a_marker`，终点误差不超过 `0.10 m`。
- `reach_checkpoint_b`：仅在 A 完成后可执行；依次经过 `[-1.25, -1.35]`、`[0.75, -1.35]`、`[1.15, -0.95]`，再到达 B。
- `inspect_top_camera`：仅在 B 完成后可执行；通过 `top_camera` 保存 RGB 和深度帧并完成任务。

黄色、绿色和品红色的无碰撞 site 分别标出 A、B 和相机检查交互点。机器人使用两个正交 slide joint 和位置执行器进行确定性平面导航；这是任务级移动控制器，不是差速轮胎或打滑模型。货架使用可碰撞 box primitive，控制器在运动与稳定阶段都检查真实 MuJoCo 接触。

## 文件

- `scene_spec.json`：规范化场景、假设、传感器、动作依赖和任务条件。
- `model.xml`：可直接编译的 MJCF 场景。
- `environment.py`：环境 API、动作校验、依赖检查、航点控制、碰撞检查和 RGB-D 采集。
- `interaction_manifest.json`：机器可读的交互点、可见标记和动作 schema。
- `physics_smoke.py`：MJCF 编译、动力学、非法动作、路线安全、碰撞、重置及 XML/MJB 重载验证。
- `render_smoke.py`：真实顶视相机 RGB-D、标记可见性、机器人像素位移、任务成功和渲染重置验证。

环境公开 `list_interaction_points()`、`get_action_schema()`、`reset(seed=None)`、`step(action)`、`observe()` 和 `is_success()`。

## 运行

从本目录执行：

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

macOS 上 `MUJOCO_GL=glfw` 使用 MuJoCo 的本地 CGL 上下文，因此命令需要可访问当前图形会话。渲染会生成 `output/screenshots/before.png`、`after.png` 和 `reset.png`，并在相邻的 `.npy` 文件中保存深度数组。

项目根目录的 `showcase/capture_sequences.py` 会额外保存每个交互状态的连续 RGB 帧、`output/screenshots/sequence.png` contact sheet 和多页 `sequence.tif`；运行它时在 macOS 同样使用 `mjpython`。

## 验证结果

静态、物理和渲染结果分别保存在 `output/scene_spec_validation.json`、`output/physics_results.json` 和 `output/render_results.json`。只有 `render_results.json` 为 `PASS` 时，才表示截图确实来自 MuJoCo、三个交互标记可见、机器人在画面中移动、完整任务成功且渲染重置已恢复初始状态。
