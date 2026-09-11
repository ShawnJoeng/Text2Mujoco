# Text2MuJoCo Showcase

[English](README.md) · [中文](README.zh-CN.md)

The canonical showcase lives in the [project README](../README.md#showcase), where each example includes its prompt, interaction chain, and full storyboard image.

- [01 - Button, Cube, and Box](01-button-cube-box)
- [02 - Smart Tool Cabinet](02-smart-drawer)
- [03 - Warehouse Navigation](03-warehouse-navigation)
- [04 - Lever and Ramp Ball](04-lever-ball-ramp)
- [05 - Robotic Arm Sorting Cell](05-robot-arm-sorting)
- [06 - Forklift Pallet Delivery](06-forklift-pallet)
- [07 - Robot Peg Assembly](07-robot-assembly)
- [08 - Conveyor-to-Arm Handoff](08-conveyor-arm)

Example prompts are recorded in [`sample_queries.json`](sample_queries.json). [`validate_manifests.py`](validate_manifests.py) checks the canonical interaction contract across all eight scenes, while [`dense_archive_test.py`](dense_archive_test.py) verifies the committed 200 ms GIF/TIFF archives. Each example directory contains its MJCF, environment API, validation scripts, reports, and generated visual evidence. The collector writes into this repository's `showcase/` tree so every persisted path stays package-relative. The root README is the canonical showcase view: it uses dense RGB animations sampled every `0.20 s` of MuJoCo simulation time, with the original short keyframe storyboards available in each example directory.
