# Interaction Patterns

An interaction point is a callable task contract backed by stable MuJoCo names, not merely a label in MJCF.

Each point must declare a unique ID, typed target, optional non-colliding marker site, pose or region, action schema, preconditions, observable success conditions, dependency IDs, effects, and reset state. Use the envelope `{"id": "press_start_button", "payload": {"press_depth_m": 0.018}}` when the actuator is position-controlled; use force units only when the MJCF actuator and contact model actually expose force semantics.

Expose:

```python
list_interaction_points() -> list[dict]
get_action_schema() -> dict
reset(seed=None) -> dict
step(action: dict) -> dict
observe() -> dict
is_success() -> bool
```

`step` must accept exactly `{"id": string, "payload": object}` and reject unknown IDs, invalid payloads, repeated actions, and unmet dependencies. `reset()` returns the initial observation. Core behavior must not depend on viewer timing. A viewer/keyboard or ROS 2 adapter may call the same API.

| Affordance | Typical MuJoCo implementation | Observable result |
| --- | --- | --- |
| press/toggle | slide or hinge joint plus actuator | joint reaches target and state changes |
| grasp | equality weld toggled through `data.eq_active`, or a gripper actuator plus contact | constraint becomes active and the payload falls when it is released in mid-air |
| pick/place | controller trajectory plus an equality weld released before settling | released object settles inside target bounds under gravity |
| push | force/actuator/contact trajectory | displacement exceeds threshold without violating bounds |
| open/close | hinge/slide actuator | joint reaches target within tolerance |
| navigate | free/planar joint or robot controller | pose error is below tolerance |
| inspect | named camera/sensor capture | requested arrays exist and pass validity checks |

## Markers Are Decals, Not Floating Balls

A marker is a named `<site>`, and a site never generates contacts — that is by design and it cannot be changed. So a marker must never be modelled as a shape a viewer would expect to be solid. An emissive sphere hovering above a table is read as an object, and then every payload and every robot link visibly passes straight through it; the scene looks broken in exactly the place it was trying to explain itself.

Make each viewer-facing marker a thin `type="cylinder"` disc painted onto the surface it annotates, with half-height equal to its height above that surface so the underside is flush:

```xml
<site name="insertion_marker" type="cylinder" pos="0.17 0.21 0.906" size="0.050 0.006"
      material="marker_magenta" group="2"/>
```

Three rules keep a decal legible. There must be a real surface directly beneath it — ray-cast downward and reject a marker over open floor the robot never reaches. Its centre must not be inside another geom, so keep a decal off the footprint of the payload it marks or it is buried and invisible. And each interaction that a render check distinguishes by colour needs its own colour.

Then sort the remaining sites by who reads them:

- a site the code uses as a kinematic reference (a tool centre, a fork tip) is not a marker: move it to `group="4"` and drop the emissive material, so it is not drawn for the viewer at all;
- a site nothing references — not the spec, not the environment, not a render check — is deleted;
- every `marker_site` named in `scene_spec.json` must still resolve to a drawn `group="2"` site.

Store the complete contract in the canonical `interaction_manifest.json` shape (`schema_version`, `backend`, `source`, `action_envelope`, `interaction_points`, and `dependency_order`); MJCF names alone cannot carry action schemas and dependency graphs.

Reports should use package-root-relative paths and must not include hostnames, home-directory paths, credentials, or other machine-local identifiers.
For action payloads that contain `output_dir`, resolve the value relative to the package root and require the resolved directory to remain inside that package. Reject empty values, POSIX/Windows absolute paths outside the package, NUL characters, and `..` escapes. Artifact persistence should use a package subdirectory such as `output/`; reject final paths or intermediate components that are symlinks, and keep MJCF and MJB destinations distinct. This keeps returned sensor paths portable and prevents an action from writing outside the generated environment.

Reset must restore `MjData`, task state, controller state, deterministic seed, and any dynamically enabled constraints — a weld left active across a reset carries the payload into the next episode. Represent a grasp with an equality constraint rather than direct `qpos` assignment; if a teleport genuinely cannot be avoided, label it as a task-level abstraction rather than physical contact validation, and do not describe the result as a grasp.
