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
    <geom contype="1" conaffinity="7"/>
  </default>
  <default class="robot_part">   <!-- links, gripper, fingers, mast, forks, wheels -->
    <geom contype="2" conaffinity="7"/>
  </default>
  <default class="payload_part"> <!-- free bodies the task moves -->
    <geom contype="4" conaffinity="7"/>
  </default>
</default>
```

Every class carries `conaffinity="7"`, so world-robot, world-payload, robot-payload, payload-payload **and robot-robot** are all tested. Do not narrow a robot class to filter self-collision. `contype="2" conaffinity="5"` looks harmless and is the single most common cause of a robot that folds through its own forearm: `2 & 5 == 0`, so no pair of robot geoms is ever tested, and the arm has no boundary against itself. The audit cannot see the defect either, because no contact is generated to report.

Switching robot-robot on turns previously invisible self-penetration into real contacts, and the links that nest at a joint — a hinge hub drawn inside the link it turns, a ram inside its sleeve — will immediately fight each other. Name those pairs, and only those, one at a time:

```xml
<contact>
  <exclude name="elbow_wrist_nest" body1="arm_link2" body2="arm_link3"/>
  <exclude name="lift_sleeve_nest" body1="arm_link3" body2="arm_lift"/>
</contact>
```

The distinction that matters is between a *declared* overlap and a *hidden* one. An overlap named in `<contact><exclude>` is a design decision a reader can check. An overlap that a contype/conaffinity mask silently swallowed is a defect nothing reports. So: accept an overlapping pair when it is a welded neighbour or a declared `<exclude>`, and reject any pair whose overlap only survives because a mask filtered it. `model.exclude_signature` holds the declarations, one `(body1 << 16) + body2` per entry, so the check is mechanical:

```python
signatures = {(int(s) >> 16, int(s) & 0xFFFF) for s in np.asarray(model.exclude_signature).ravel()}
declared = (body_a, body_b) in signatures or (body_b, body_a) in signatures
```

Measure the geometry itself with `mujoco.mj_geomDistance(model, data, geom_a, geom_b, distmax, None)`, which returns a signed distance whether or not the pair is filtered. If a *non-adjacent* pair now collides — a closed jaw clipping into the forearm capsule above it — that is a genuine defect, not a pair to exclude. Change the geometry or tighten the joint range, and write the arithmetic into a comment next to the range.

MuJoCo already excludes direct parent-child body pairs and any pair whose bodies are both welded to the world, so static scenery never reports contact against other static scenery. Overlapping static geoms are therefore invisible to a contact audit and have to be checked by arithmetic: a pedestal that starts at `z=0` and a table slab at the same `x, y` interpenetrate in every rendered frame without a single contact being generated.

A visual-only geom is acceptable only for detail that lies inside the collidable envelope of the same body, such as a hub, a stripe, or a heading marker. If a geom is the outermost surface in any direction another body can approach from, it collides.

## Drawn Axes and Link Continuity

A joint is a coordinate, not a shape. A slide joint whose moving body carries geometry but whose *travel* carries none opens a gap under the parent link that grows with the joint value — at full extension a 0.165 m stroke leaves the hand hanging 165 mm below the forearm with nothing in between. It reads in a render as the gripper having fallen off before it was attached, and no contact, pose, or task assertion sees it, because kinematically the chain is intact.

Give the axis a body of its own and two geoms that always overlap: a sleeve fixed to the parent that covers the travel, and a ram on the moving body that stays inside the sleeve at every joint value. Check both end stops by arithmetic, then declare the sleeve/ram pair with `<exclude>`:

```xml
<geom name="lift_sleeve" class="robot_part" type="cylinder" pos="0.16 0 0.012" size="0.048 0.052" mass="0.22"/>
<body name="arm_lift" pos="0.16 0 -0.075" gravcomp="1">
  <joint name="tool_z" type="slide" axis="0 0 1" range="-0.17 0.045" damping="3.0" armature="0.008"/>
  <geom name="lift_ram" class="robot_part" type="cylinder" pos="0 0 0.125" size="0.026 0.125" mass="0.20"/>
```

The check to run is continuity, not contact: for every jointed body, measure the widest gap between it and the nearest drawn ancestor over the whole replay, and reject anything above `5e-3`. A seam that is closed at both end stops but opens in between still renders as a broken arm.

## Declared Mass

A geom with neither `mass` nor `density` compiles at density 1000 — water. A decorative 0.058 m hinge hub then weighs 1.1 kg and can outweigh the entire arm it decorates, which changes every inertia, every servo gain, and every settling time in the scene while looking like a cosmetic detail. Put an explicit `mass` on every geom of every jointed body, and sanity-check the total against `model.body_mass`.

## Servos That Hold Still

A position servo holding a load against gravity settles at a steady-state error of `weight / kp`, and that error is a droop the render shows. A 0.08 kg fingertip on a `kp="420"` lift servo sags exactly 1.87 mm; the same servo carrying a 2 kg tool column sags 47 mm. Command every position servo to hold `qpos0` for two seconds and reject more than 2 mm or 1 degree of drift.

Two fixes, in order of preference: raise `kp` until the droop is below tolerance, or add `gravcomp="1"` to the bodies the axis carries. `gravcomp` models a real counterbalance and is the honest choice for a lift column, but it must be on **every** carried body including the leaves — a fingertip left without it reintroduces the whole droop on its own. Never absorb the droop by moving the geometry down to meet it, because every clearance measured against the nominal pose then becomes wrong.

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

## Grasps as Constraints

Carrying a payload by writing its free-joint `qpos` every step is not a grasp. Nothing can make the payload slip, releasing it is a teleport, and a part lying on the bench is indistinguishable from a part in the hand — which is why a dropped part keeps being treated as the object under manipulation. Use an equality constraint that is declared inactive and toggled at runtime:

```xml
<equality>
  <weld name="peg_grasp" body1="arm_tool" body2="red_peg" relpose="0 0 -0.125 1 0 0 0"
        active="false" solref="0.01 1" solimp="0.96 0.99 0.001"/>
</equality>
```

A weld's `eq_data` row is `[anchor(3), relpose_pos(3), relpose_quat(4), torquescale(1)]`. Measure the actual tool-to-payload offset at the instant the jaws close and write it in, so the constraint engages already satisfied — no jolt, no snap into place:

```python
tool_frame = data.xmat[tool_body].reshape(3, 3)
relative = tool_frame.T @ (data.xpos[payload_body] - data.xpos[tool_body])
mujoco.mju_mat2Quat(quat, np.ascontiguousarray(tool_frame.T @ data.xmat[payload_body].reshape(3, 3)).reshape(9))
model.eq_data[eq_id, 3:6] = relative
model.eq_data[eq_id, 6:10] = quat
data.eq_active[eq_id] = 1          # 0 to let go
```

Close the jaws *onto* the payload — a small negative clearance, e.g. an inner face 1.5 mm inside the payload radius — rather than around a gap the constraint spans invisibly. Then delete every `qpos`/`qvel` write that fakes a carry, and every "resolve the final pose" assignment that fakes a release; let the payload settle under gravity and contacts instead. Do the same for the robot's own coordinates: a `data.qpos[joint] = target` write papers over actuator tracking error and makes the servo tuning untestable.

The regression that proves the grasp is real is a mid-air release: grasp, carry, drop the constraint with the payload still above the surface, and require it to fall at least 20 mm. A welded payload falls; a `qpos`-driven one hangs in the air.

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
