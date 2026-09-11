# Text2MuJoCo 展示

[English](README.md) · [中文](README.zh-CN.md)

规范的展示位于 [项目 README](../README.md#showcase)，其中每个示例都包含它的提示词、交互链和完整的故事板图片。

- [01 - 按钮、方块与开口盒](01-button-cube-box)
- [02 - 智能工具柜](02-smart-drawer)
- [03 - 仓库导航](03-warehouse-navigation)
- [04 - 拉杆与斜坡小球](04-lever-ball-ramp)
- [05 - 机械臂分拣单元](05-robot-arm-sorting)
- [06 - 叉车托盘配送](06-forklift-pallet)
- [07 - 机器人定位销装配](07-robot-assembly)
- [08 - 输送带到机械臂的交接](08-conveyor-arm)

示例提示词记录在 [`sample_queries.json`](sample_queries.json) 中。[`validate_manifests.py`](validate_manifests.py) 会在全部八个场景上检查规范的交互契约，[`dense_archive_test.py`](dense_archive_test.py) 则校验已提交的 200 ms GIF/TIFF 归档。每个示例目录都包含它的 MJCF、环境 API、校验脚本、报告和生成的视觉证据。收集器把内容写入本仓库的 `showcase/` 目录树，因此每一条持久化路径都保持包内相对。根 README 是规范的展示视图：它使用每 `0.20 s` MuJoCo 仿真时间采样一次的密集 RGB 动画，原始的短关键帧故事板保留在各个示例目录中。
