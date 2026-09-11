# Text2MuJoCo 端到端测试

[English](README.md) · [中文](README.zh-CN.md)

这是针对以下需求生成的软件包：

> 在桌面上放置一个按钮、一个红色立方体和一个开口箱。按下按钮后，抓取立方体并将其放入箱中。用 RGB-D 相机观察结果。

按钮使用真实的滑动关节与位置执行器；其动作以米为单位使用 `press_depth_m`，与执行器的位置目标一致。立方体使用自由关节。抓取是一个名为 `cube_grasp` 的 `<weld>` 相等约束，声明时处于关闭状态，在运行时开合；需求中未包含机器人，因此它按拾取瞬间测得的偏移把立方体锚定到 world 上。释放会关闭该焊接约束，之后的重力、接触与静置沉降由 MuJoCo 仿真。容器由五个独立的碰撞体组成，内部保持中空。

本目录中的检查针对这个轴对齐的按钮/立方体/开口箱示例。新的软件包应针对自身的资源与交互条件重新生成运行时处理器与检查。场景以声明的抓取位姿与给定的放置高度作为释放目标；最终的静止位姿由物理决定。前三个交互点由物理检查覆盖，检查点由 RGB-D 渲染检查覆盖。

本测试目录依赖同级的 `../../text2mujoco_codex/scripts/validate_scene_spec.py`；请在此处展示的仓库布局中运行它。单独复制本测试目录得到的是一份依赖仓库结构的副本，完整的技能安装需要保留上述布局。

## 运行

需要 Python 3.9 或更高版本。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
./run_all.sh
```

`run_all.sh` 首先以 `MUJOCO_GL=disable` 运行静态检查与物理检查。在 macOS 上，自动渲染通过 `MUJOCO_GL=glfw` 使用 MuJoCo 原生的 CGL 上下文。在 Linux 上，它先在一个进程中尝试 EGL，若不可用则在一个新进程中尝试 OSMesa。Linux 可能需要对应的运行时包，通常是 EGL 所需的 `libegl1` 或 CPU 渲染所需的 `libosmesa6`。

如需显式选择：

```bash
MUJOCO_RENDER_BACKEND=osmesa ./run_all.sh
```

## 证据

该套件验证：

- 场景规范校验以及校验器的负例分支；
- 清单与规范的一致性，以及带类型的 MuJoCo 目标解析；
- MJCF XML 解析、模型编译，以及具名的刚体/geom/关节/站点/相机；
- box/cylinder 的完整尺寸到 MJCF size 的转换；
- 按钮执行器运动、自由刚体的重力、五部件开口箱的接触、静置沉降与重置；
- MJCF 与 MJB 的重新加载；
- 依赖与 payload 的失败分支，以及合法的交互序列；
- 真实 MuJoCo 的前后 RGB、深度数组、图像差异、红色立方体可见性与像素位移。

预期的仿真器图像为 `output/screenshots/initial/rgb.png` 与 `output/screenshots/final/rgb.png`。项目级收集器会写出一份有节奏的关键帧故事板，包含 `sequence.gif`、拼版图与 `sequence.tif`，另有可选的密集 RGB 序列。关键帧 GIF 是离散已验证状态的动画，粒度停留在这些状态上；TIFF 是全分辨率的关键帧归档。报告与 XML 预览属于文本产物；请以渲染器生成的图像文件作为仿真器证据。

在仓库根目录采集密集序列：

```bash
MUJOCO_GL=glfw mjpython showcase/capture_sequences.py --dense --scene 01-button-cube-box
```

密集 RGB 帧默认每 `0.20 s` MuJoCo 仿真时间采样一次。使用 `--dense-interval <seconds>` 可修改该间隔。`output/screenshots/dense_sequence.gif` 以 `200 ms` 延迟播放普通帧，最后一帧为 `800 ms`；`output/screenshots/dense_sequence.tif` 保存同一批全分辨率 RGB 页面。`output/dense_sequence_results.json` 记录仿真时间戳以及从 1 开始的 GIF/TIFF `archive_frame` 映射。深度数据在 `output/screenshots/sequence/` 下经过验证的关键帧采集中提供。

## 已验证运行 (2026-09-08)

已提交的物理与渲染报告记录了通过的 MJCF 编译、执行器运动、重力释放、开口箱接触、有限状态、失败分支、确定性重置、RGB-D 采集、可见的立方体位移，以及完整的四步动作任务。请用你机器上可用的后端重新运行上述命令以复现这些检查。`model.xml` 是可移植的源文件；请为每个目标操作系统或架构重新生成 `model.mjb`，平台相关的二进制缓存保持本地生成。
