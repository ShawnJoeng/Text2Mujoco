# MuJoCo Runtime Patterns

## Installation and Backend Selection

Use the maintained Python package, preferably with a tested pin:

```bash
python -m pip install "mujoco==3.2.7" numpy pillow
```

Do not install `mujoco-py`. The wheel includes the MuJoCo engine. Linux rendering still needs a usable EGL or OSMesa runtime.

Select the backend before importing `mujoco` or OpenGL:

```bash
MUJOCO_GL=disable python physics_smoke.py
MUJOCO_GL=egl python render_smoke.py
MUJOCO_GL=osmesa python render_smoke.py
```

Run EGL and OSMesa probes as separate processes. EGL is preferred for NVIDIA GPU rendering; OSMesa is a CPU fallback for machines without graphics exposure. GLFW generally requires a display server on Linux. On macOS, use MuJoCo's `mjpython` with `MUJOCO_GL=glfw` and an active graphics session for the native CGL context; record the actual context in the test result rather than calling it EGL.

## Loading and Stepping

```python
import mujoco

model = mujoco.MjModel.from_xml_path("model.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
for _ in range(100):
    mujoco.mj_step(model, data)
```

Resolve names and fail when an object is missing:

```python
def require_id(model, object_type, name):
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"missing MuJoCo object: {name}")
    return object_id
```

Use `model.jnt_qposadr[joint_id]` and `model.jnt_dofadr[joint_id]`; never assume a free joint's address from declaration order. A free-joint qpos is translation followed by a `wxyz` quaternion.

Reset both simulator and task state:

```python
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)
```

## Geometry Conversion

- `box dimensions [x, y, z]` -> `size="x/2 y/2 z/2"`
- `cylinder dimensions [d, d, h]` -> `size="d/2 h/2"`
- `sphere dimensions [d, d, d]` -> `size="d/2"`
- `capsule dimensions [d, d, length]` -> state whether length is total or cylindrical before conversion

Keep an open receptacle as separate bottom and wall geoms. Collision filtering is determined by `contype` and `conaffinity`; debug sites should not be collision geoms.

## Offscreen RGB-D

```python
renderer = mujoco.Renderer(model, height=480, width=640)
try:
    renderer.update_scene(data, camera="overview")
    rgb = renderer.render().copy()
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="overview")
    depth = renderer.render().copy()
finally:
    renderer.close()
```

Verify array shape, dtype, finite depth pixels, dynamic range, and task-relevant color/position changes before accepting a screenshot. Pixel-exact hashes are inappropriate across EGL and OSMesa.

## Persistence

MJCF source remains the editable artifact. After compile, an optional binary can be saved and reopened:

```python
mujoco.mj_saveModel(model, "model.mjb", None)
reloaded = mujoco.MjModel.from_binary_path("model.mjb")
```

Reload the XML with `MjModel.from_xml_path` as a separate assertion. Simulator qpos changes are runtime state and do not rewrite the source MJCF; save episode state separately when needed.
