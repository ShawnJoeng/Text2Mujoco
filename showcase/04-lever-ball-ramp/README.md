# Lever Ball Ramp Workbench

这是一个 MuJoCo 3.2.7 生成场景：拉下蓝色杠杆，黄色挡板升起，紫色小球沿斜坡滚入绿色目标托盘，最后由固定 RGB-D 相机检查结果。四个交互点都在 MJCF 中有可见 site 标记，并在 `interaction_manifest.json` 中声明了目标类型、动作 schema、依赖、效果和 reset 状态。

## 交互顺序

`pull_blue_lever` -> `check_release_zone` -> `confirm_target_tray` -> `inspect_with_camera`

杠杆和挡板使用真实 hinge/slide joint 与 position actuator。球使用 free joint；释放后的位置、碰撞、滚动和稳定均由 MuJoCo 物理计算。释放区和目标区交互会推进物理并检查球的位置/速度，`inspect_with_camera` 保存 RGB 与深度数组。

## 运行

```bash
/tmp/text2mujoco-local-env/bin/python ../../text2mujoco/scripts/validate_scene_spec.py scene_spec.json --json
MUJOCO_GL=disable /tmp/text2mujoco-local-env/bin/python physics_smoke.py
MUJOCO_GL=glfw /tmp/text2mujoco-local-env/bin/python render_smoke.py
```

渲染命令在 macOS 使用 MuJoCo 原生 CGL 上下文。若当前主机没有可用图形上下文，`render_smoke.py` 会保存失败报告而不会伪造截图；Linux 可在新进程中分别尝试 `MUJOCO_GL=egl` 与 `MUJOCO_GL=osmesa`。

## 输出

默认生成 `physics_results.json`、`render_results.json`，以及渲染成功时的 `screenshots/before/rgb.png`、`screenshots/after/rgb.png` 和对应 `depth.npy`。`scene_spec.json` 是规范输入，`model.xml` 是可编辑 MJCF 源文件。
