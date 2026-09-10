# Robot and Manipulator Patterns

Use this reference when a request includes a robot, arm, gripper, handoff, or
coordinated object manipulation. The robot must be represented by named MuJoCo
bodies, joints, sites, and actuators; a colored mesh or a free-floating body is
not an executable robot.

## Model Boundary

- Prefer a small, explicit primitive robot when no robot asset is supplied.
  A planar or tabletop arm may use revolute joints with position actuators and
  a named end-effector site. State the simplification in `assumptions`.
- Every body of the robot collides, including against the rest of the robot.
  Links, gripper palm, fingers, mast, carriage, forks, wheels, and mobile base
  each need at least one geom with a nonzero `contype`/`conaffinity`, because the
  robot body is the part a viewer watches and a visual-only chain sweeps through
  tables, bins, and racks with nothing to report. Give every collision class
  `conaffinity="7"` so `robot<->robot` is tested like every other pair, and name
  the handful of pairs that are designed to nest in `<contact><exclude>` — see
  [mujoco_patterns.md](mujoco_patterns.md). Never suppress self-collision with a
  narrowed mask (`contype="2" conaffinity="5"`) and never by zeroing both flags:
  both leave the arm with no boundary against itself and generate no contact for
  an audit to find.
- A gripper needs at least one named joint/actuator and a visible fingertip or
  grasp site. Size the closed aperture against the target: with fingers mounted
  at `±mount` and half-thickness `t`, the closed inner face sits at
  `mount - stroke - t`, and it should land just *inside* the object's half-width
  so the jaws close onto the payload rather than around a gap. A closed pose that
  clears the object by several millimetres is what makes a task-level attachment
  look like the thing doing the work.
- A mobile manipulator should separate base joints, lift/arm joints, and the
  object free joint. Keep each actuator's range and units explicit.
- For `robot` or `mjcf_include` assets, declare typed names in `exports` and
  confirm every exported body, joint, actuator, site, and camera after MJCF
  compilation. Do not claim that an unresolved URDF is directly loadable MJCF.

## The Arm Has To Be Drawn, Not Only Jointed

Three defects look identical in a render — the hand has fallen off the arm — and
none of them is a topology error. Check all three before touching the kinematics:

- **An undrawn prismatic axis.** A lift or extension joint whose travel carries
  no geometry opens a gap under the parent link proportional to the joint value.
  Draw the axis as a sleeve on the parent plus a ram on the moving body that
  stays inside it at both end stops.
- **A servo drooping under its load.** Steady-state error is `weight / kp`, so a
  soft lift servo lowers the whole tool column a visible distance below where the
  model says it is. Raise `kp`, or add `gravcomp="1"` to every body the axis
  carries — the leaf fingertips included.
- **An undeclared mass.** A geom with no `mass` and no `density` compiles at
  density 1000, so a decorative hub can outweigh the arm and drag it down.

The measurement that covers all three is link continuity: for every jointed body,
the widest gap to its nearest drawn ancestor across the whole replay, rejected
above 5 mm. Endpoint pose checks pass while the seam is open in the middle.

## Enough Wrist To Put Something Down

An arm whose only vertical freedom is one lift axis cannot release a part and
withdraw: retracting drags the tool column back along the path it came down, and
tipping the base sends the whole hand into the fixture. Give the hand at least one
more DOF than the task strictly needs — a wrist pitch hinge with its own position
actuator is usually enough — and then *use it in the scripted sequence*: tip the
hand forward on approach, tip it back to lift away from a part just placed. An
extra joint that no interaction commands is decoration. Check by arithmetic that
the tilted pose does not drive the tool into scenery, and pitch only where the
clearance exists.

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
A weld holding the payload to the tool does not excuse the path, because the
welded body is dragged through whatever the tool passes through.

## Synchronized Manipulation

An interaction sequence should expose the causal stages instead of one opaque
teleport action. A typical contract is:

`home_or_approach -> close_gripper -> transport -> open_gripper -> verify`

Each stage must have a stable ID, a typed target, dependencies, preconditions,
observable effects, and reset data. Carry the payload with an equality weld that
is toggled through `data.eq_active` and whose `relpose` is written from the
measured offset at the instant the jaws close, so the constraint engages already
satisfied. On release, deactivate the constraint before stepping gravity and
contacts so the object settles physically instead of being assigned a final pose.
The regression that separates a held part from a part merely drawn in the right
place is a mid-air release: drop the weld with the payload above the surface and
require it to fall.

For coordinated actions, record the robot pose, object pose, gripper state, and
constraint state in `observe()`. `is_success()` must require both the robot end
state and the object goal state; an arm reaching a waypoint alone is not a
successful pick-and-place, and neither is a part resting in the target while the
tool is still buried in the fixture.

## Controller Choices

- Use joint-space position targets for small deterministic demonstrations, and
  advance MuJoCo for enough steps to reach each target. Check joint limits and
  finite state after every segment.
- If a full inverse-kinematics or trajectory controller is not implemented,
  label the controller as a task-level abstraction and keep the physical
  portion (gravity, contacts, settling, and actuator motion) genuine.
- Do not overwrite an object's `qpos` to carry it; use the equality weld. Do not
  overwrite the robot's own joint coordinates either — a `data.qpos[joint] =
  target` write hides the actuator tracking error the servo gains exist to
  control, and every clearance then gets measured against a pose the arm never
  actually reaches. If a documented teleport abstraction is genuinely
  unavoidable, expose `attachment_mode` in assumptions, do not call it a grasp,
  and test release separately.
- A conveyor or mobile base has to be driven, not narrated. Integrating a
  parcel's position in Python while a belt actuator spins for the camera means
  the parcel is not on the belt at all: no friction, no slip, and it will pass
  through a stopper. Include the actuator in the dependency graph and validate
  that handoff timing leaves the object within the receiving workspace.

## Evidence

Physics checks should compile the actual MJCF, resolve all robot and object
names, exercise an invalid dependency/action, run the complete sequence, and
verify reset. A run that only compares start and end poses cannot see the middle
of a transfer, so also assert the deepest contact over every step of the
sequence and report it as a number. Add the mid-air release regression: grasp,
carry, deactivate the weld above the surface, and require the payload to fall at
least 20 mm. Also run the model-level audit — collision coverage, declared mass,
servo hold, link continuity, marker grounding, start pose, sequence contact, and
undeclared self-overlap — and treat a failing check as a scene defect to fix, not
a threshold to loosen. Render checks should verify the end-effector,
gripper, object, and task markers in the camera image; a global color count is
insufficient when other scene objects share the same color. Dense captures may
sample RGB every `0.20 s` of simulation time, while RGB-D keyframes retain the
corresponding depth arrays.
