# Robot and Manipulator Patterns

Use this reference when a request includes a robot, arm, gripper, handoff, or
coordinated object manipulation. The robot must be represented by named MuJoCo
bodies, joints, sites, and actuators; a colored mesh or a free-floating body is
not an executable robot.

## Model Boundary

- Prefer a small, explicit primitive robot when no robot asset is supplied.
  A planar or tabletop arm may use revolute joints with position actuators and
  a named end-effector site. State the simplification in `assumptions`.
- Every body of the robot collides. Links, gripper palm, fingers, mast,
  carriage, forks, wheels, and mobile base each need at least one geom with a
  nonzero `contype`/`conaffinity`, because the robot body is the part a viewer
  watches and a visual-only chain sweeps through tables, bins, and racks with
  nothing to report. Suppress self-collision with collision classes or
  `<contact><exclude>` as described in
  [mujoco_patterns.md](mujoco_patterns.md), never by zeroing both flags.
- A gripper needs at least one named joint/actuator and a visible fingertip or
  grasp site. Size the closed aperture against the target: with fingers mounted
  at `±mount` and half-thickness `t`, the closed inner face sits at
  `mount - stroke - t` and must stay just outside the object's half-width.
  A closed pose that would drive the fingers through the object is a modeling
  error even when a task-level attachment carries the object.
- A mobile manipulator should separate base joints, lift/arm joints, and the
  object free joint. Keep each actuator's range and units explicit.
- For `robot` or `mjcf_include` assets, declare typed names in `exports` and
  confirm every exported body, joint, actuator, site, and camera after MJCF
  compilation. Do not claim that an unresolved URDF is directly loadable MJCF.

## Reachable Poses and Carry Paths

Work out the tool geometry before choosing a waypoint. The end-effector site is
usually not the contact point: fingers hang below the wrist, and forks reach
ahead of the mast. Derive each pose from the object and the support instead of
from the site, then confirm by arithmetic that the whole tool clears the
scenery it passes over.

A pose that is clear at both ends does not make the motion between them clear.
Joint-space interpolation between two valid poses bows the tool through an
arbitrary arc, which is how a gripper ends up under a tabletop halfway through
a transfer. Two options keep the path honest:

- Interpolate in Cartesian space: split the segment into small steps, solve for
  each intermediate point, and command the actuators along the way.
- Declare explicit waypoints and lift high enough to clear the tallest thing in
  between: `approach above -> descend -> grasp -> lift -> traverse -> descend ->
  release`. The carried object's lowest surface, not the tool frame, is what has
  to clear each wall and rim.

Whichever is used, the constraint to verify is the same: at every step of the
documented sequence, no geom pair interpenetrates by more than solver softness.
When a task-level attachment pins the payload to the tool, the pin does not
excuse the path, because the pinned body is dragged through whatever the tool
passes through.

## Synchronized Manipulation

An interaction sequence should expose the causal stages instead of one opaque
teleport action. A typical contract is:

`home_or_approach -> close_gripper -> transport -> open_gripper -> verify`

Each stage must have a stable ID, a typed target, dependencies, preconditions,
observable effects, and reset data. During transport, verify that the held
object follows the end-effector (through contact/constraint state or a clearly
labelled task-level attachment). On release, remove the attachment before
stepping gravity and contacts so the object can settle physically.

For coordinated actions, record the robot pose, object pose, gripper state, and
contact/attachment state in `observe()`. `is_success()` must require both the
robot end state and the object goal state; an arm reaching a waypoint alone is
not a successful pick-and-place.

## Controller Choices

- Use joint-space position targets for small deterministic demonstrations, and
  advance MuJoCo for enough steps to reach each target. Check joint limits and
  finite state after every segment.
- If a full inverse-kinematics or trajectory controller is not implemented,
  label the controller as a task-level abstraction and keep the physical
  portion (gravity, contacts, settling, and actuator motion) genuine.
- Do not silently overwrite object `qpos` while claiming a contact grasp. If
  direct `qpos` attachment is used, expose `attachment_mode` in assumptions and
  test release separately.
- For a conveyor or mobile base, include the conveyor/base actuator in the
  dependency graph and validate that handoff timing leaves the object within
  the receiving workspace.

## Evidence

Physics checks should compile the actual MJCF, resolve all robot and object
names, exercise an invalid dependency/action, run the complete sequence, and
verify reset. A run that only compares start and end poses cannot see the middle
of a transfer, so also assert the deepest contact over every step of the
sequence and report it as a number. Render checks should verify the end-effector,
gripper, object, and task markers in the camera image; a global color count is
insufficient when other scene objects share the same color. Dense captures may
sample RGB every `0.20 s` of simulation time, while RGB-D keyframes retain the
corresponding depth arrays.
