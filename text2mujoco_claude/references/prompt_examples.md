# Prompt Examples

Use a short natural-language request and let the skill preserve assumptions in the generated spec.

## Manipulation

> On a 1 m table, place a red cube beside a button. After pressing the button, grasp the cube, release it inside an open bin, and capture RGB-D before and after.

Expected output: an MJCF model with a real button joint/actuator, a free-joint cube, separate bin wall geoms, named interaction sites, a camera sensor, and an executable dependency sequence.

## Articulation

> Build a drawer that opens 0.25 m when the user sends an open action, then closes on reset. Report the joint position and an overhead image.

Expected output: a slide joint with limits and actuator, typed joint interaction target, reset state, and a camera observation contract.

## Navigation

> Create a small room with a planar mobile base and two marked waypoints. Move to waypoint B only after waypoint A is reached.

Expected output: collision-enabled room geometry, a documented planar/free base, typed waypoint sites, dependency metadata, and a distance-based success predicate.

## Robot Manipulation

> Build a tabletop robot arm with a two-finger gripper. Move to a blue block,
> close the gripper, carry it around an obstacle, release it in a red tray, and
> verify the placement with a fixed RGB-D camera.

Expected output: named arm joints and actuators, an end-effector and fingertip
contract, a movable block, a documented contact or task-level attachment mode,
ordered approach/grasp/transport/release/verify interactions, and robot/object
state checks.
