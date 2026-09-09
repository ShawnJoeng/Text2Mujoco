# Robot and Manipulator Patterns

Use this reference when a request includes a robot, arm, gripper, handoff, or
coordinated object manipulation. The robot must be represented by named MuJoCo
bodies, joints, sites, and actuators; a colored mesh or a free-floating body is
not an executable robot.

## Model Boundary

- Prefer a small, explicit primitive robot when no robot asset is supplied.
  A planar or tabletop arm may use revolute joints with position actuators and
  a named end-effector site. State the simplification in `assumptions`.
- A gripper needs at least one named joint/actuator and a visible fingertip or
  grasp site. Use collision-enabled fingertips when contact is part of the
  requested behavior; use a documented task-level attachment only when a real
  contact controller was not requested.
- A mobile manipulator should separate base joints, lift/arm joints, and the
  object free joint. Keep each actuator's range and units explicit.
- For `robot` or `mjcf_include` assets, declare typed names in `exports` and
  confirm every exported body, joint, actuator, site, and camera after MJCF
  compilation. Do not claim that an unresolved URDF is directly loadable MJCF.

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
verify reset. Render checks should verify the end-effector, gripper, object,
and task markers in the camera image; a global color count is insufficient when
other scene objects share the same color. Dense captures may sample RGB every
`0.20 s` of simulation time, while RGB-D keyframes retain the corresponding
depth arrays.
