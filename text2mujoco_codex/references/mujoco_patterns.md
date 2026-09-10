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

## Collision Classes

A pair of geoms is tested only when `(contype1 & conaffinity2) || (contype2 & conaffinity1)`. Setting `contype="0" conaffinity="0"` removes a geom from collision entirely: it is drawn, it carries mass, and it passes through everything. That is the usual reason a rendered arm sweeps through a table while every contact assertion still passes, so it must never be the way an unwanted pair is suppressed. Give each geom a class instead and keep the classes in a `<default>` block so the intent is visible:

```xml
<default>
  <default class="world_part">   <!-- tables, bins, walls, rails, belts -->
    <geom contype="1" conaffinity="6"/>
  </default>
  <default class="robot_part">   <!-- links, gripper, fingers, mast, forks, wheels -->
    <geom contype="2" conaffinity="5"/>
  </default>
  <default class="payload_part"> <!-- free bodies the task moves -->
    <geom contype="4" conaffinity="7"/>
  </default>
</default>
```

That yields world-robot, world-payload, robot-payload, and payload-payload collisions while filtering robot self-collision, which a short primitive chain does not need. When one specific pair must be suppressed, say so explicitly and keep everything else live:

```xml
<contact>
  <exclude body1="gripper" body2="held_part"/>
</contact>
```

MuJoCo already excludes direct parent-child body pairs and any pair whose bodies are both welded to the world, so static scenery never reports contact against other static scenery. Overlapping static geoms are therefore invisible to a contact audit and have to be checked by arithmetic: a pedestal that starts at `z=0` and a table slab at the same `x, y` interpenetrate in every rendered frame without a single contact being generated.

A visual-only geom is acceptable only for detail that lies inside the collidable envelope of the same body, such as a hub, a stripe, or a heading marker. If a geom is the outermost surface in any direction another body can approach from, it collides.

## Resting Poses and Initial Contact

Seat a resting body by half-size rather than by eye: `z_center = support_top_z + half_height`, where `support_top_z` is the support body's world z plus its own half-height. Reusing a support's center z, or copying a neighbor's z that sits on a different support, is the usual cause of a body starting inside its table, pad, or belt.

Audit initial contact after the first `mj_forward` and before stepping:

```python
mujoco.mj_forward(model, data)
overlaps = [
    (
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[index].geom1),
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[index].geom2),
        float(data.contact[index].dist),
    )
    for index in range(data.ncon)
    if data.contact[index].dist < -1e-4
]
```

`contact.dist` is the signed gap, so a negative value means the geoms interpenetrate. A body resting exactly on its support reports about `0.0`. Small negative values after stepping are normal solver softness; at `t=0` they are a modeling error and must be fixed in the pose, not absorbed by the solver.

A `t=0` audit only describes the start pose. To cover the motion, wrap the module-level step function once and keep the deepest contact of the whole run:

```python
deepest, genuine = {"dist": 0.0, "pair": None}, mujoco.mj_step

def watched(model, data, *args, **kwargs):
    genuine(model, data, *args, **kwargs)
    for index in range(data.ncon):
        if data.contact[index].dist < deepest["dist"]:
            deepest.update(dist=float(data.contact[index].dist),
                           pair=(data.contact[index].geom1, data.contact[index].geom2))

mujoco.mj_step = watched   # restore in a finally block
```

Patch `mujoco.mj_step` rather than an environment method, because helpers such as a settle loop or a gripper controller usually call the module function directly.

## Stops, Strokes, and the Timestep Floor

A joint `range` is a soft constraint, not a wall, so the declared stroke is not the stroke the body actually travels. The default `jnt_solref` of `[0.02 1]` is loose enough that a light actuated part coasts well past its own stop: a 0.04 kg button cap with `range="0 0.012"` and `damping="3"`, driven by a `kp="450"` position servo, peaked at `qpos = 0.015345` — 3.345 mm beyond a 12 mm stop, far enough to strike a backplate seated 2 mm behind full travel. Stiffen the limit toward the `2 * timestep` floor and raise the damping until the declared stroke is the real one:

```xml
<joint name="unlock_button_slide" type="slide" axis="0 0 -1" range="0 0.012"
       damping="8" solreflimit="0.004 1" solimplimit="0.99 0.999 0.0005"/>
```

At `timestep="0.002"` that took the overshoot to 0.677 mm, and the raised damping took it to 0.025 mm. Do not instead move the housing back to clear the overrun: that hides a stroke the model claims it does not have, and every clearance measured against the declared range stays wrong.

The same floor bounds contact stiffness. A contact's time constant cannot go below `2 * timestep`, so at `timestep="0.002"` the stiffest reachable contact still let a 0.2 kg cube dropped 0.25 m compress about 1.372 mm; halving the step let `solref="0.002 1"` resolve the identical impact inside 0.818 mm. When an impact transient trips the audit, lower the timestep rather than raise the tolerance — the tolerance is what tells you the geometry is sound.

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
