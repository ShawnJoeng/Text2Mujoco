# 智能桌面工具柜（MuJoCo 3.2.7）

此包实现以下顺序任务：按下绿色按钮解锁，将工具抽屉沿前方拉开 `0.22 m`，再从固定相机采集 RGB-D 并确认抽屉已打开。

## 场景与交互

- `press_unlock_button`：驱动 `unlock_button_slide` 到 `0.012 m`。黄色发光 site 标出解锁点。
- `pull_drawer_22cm`：只在解锁后可执行，位置执行器驱动 `drawer_slide` 到 `0.22 m`。青色发光 site 标出把手。
- `inspect_open_drawer`：只在抽屉位置不小于 `0.218 m` 时可执行，使用 `fixed_inspection_camera` 采集 `640 x 480` RGB-D。品红色发光 site 标出检查点。

抽屉使用真实 slide joint、位置执行器和多块碰撞几何；拉动动作是任务级控制指令，不代表机器人手爪接触验证。场景仅使用 MJCF primitive，不依赖外部 mesh 或下载资源。

## 文件

- `scene_spec.json`：规范化场景、假设、传感器和动作契约。
- `model.xml`：可直接编译的 MJCF。
- `environment.py`：无全局模拟器状态的环境 API。
- `interaction_manifest.json`：机器可读的交互顺序及可见标记映射。
- `physics_smoke.py`：编译、动力学、动作拒绝分支、22 cm 行程、重置和 MJB/XML 重载检查。
- `render_smoke.py`：真实固定相机 RGB-D、三种标记可见性和把手像素位移检查。

环境公开 `list_interaction_points()`、`get_action_schema()`、`reset(seed=None)`、`step(action)`、`observe()` 和 `is_success()`。

## 运行

从本目录执行：

```bash
python3 ../../text2mujoco_codex/scripts/validate_scene_spec.py scene_spec.json
MUJOCO_GL=disable python3 physics_smoke.py
MUJOCO_GL=glfw mjpython render_smoke.py
```

macOS 上 `MUJOCO_GL=glfw` 使用 MuJoCo 的本地 CGL 上下文。渲染成功时，关闭/打开截图分别写入 `output/screenshots/before.png` 和 `output/screenshots/after.png`，完整 RGB-D 帧位于相邻的 `before_capture/`、`after_capture/` 目录。

项目根目录的 `showcase/capture_sequences.py` 会额外保存每个交互状态的连续 RGB 帧、`output/screenshots/sequence.png` contact sheet 和多页 `sequence.tif`；运行它时在 macOS 同样使用 `mjpython`。

## 验证边界

静态、物理和渲染结果分别保存在 `output/scene_spec_validation.json`、`output/physics_results.json` 和 `output/render_results.json`。只有 `render_results.json` 为 PASS 时，才表示固定相机、可见标记和画面中的抽屉运动已经真实验证。
