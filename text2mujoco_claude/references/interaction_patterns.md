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
| grasp | gripper actuator/contact or equality constraint | attachment/contact state becomes active |
| pick/place | controller trajectory and release; documented qpos abstraction for lightweight tasks | released object settles inside target bounds |
| push | force/actuator/contact trajectory | displacement exceeds threshold without violating bounds |
| open/close | hinge/slide actuator | joint reaches target within tolerance |
| navigate | free/planar joint or robot controller | pose error is below tolerance |
| inspect | named camera/sensor capture | requested arrays exist and pass validity checks |

Represent optional markers as named `<site>` elements with no collision role. Store the complete contract in the canonical `interaction_manifest.json` shape (`schema_version`, `backend`, `source`, `action_envelope`, `interaction_points`, and `dependency_order`); MJCF names alone cannot carry action schemas and dependency graphs.

Reports should use package-root-relative paths and must not include hostnames, home-directory paths, credentials, or other machine-local identifiers.
For action payloads that contain `output_dir`, resolve the value relative to the package root and require the resolved directory to remain inside that package. Reject empty values, POSIX/Windows absolute paths outside the package, NUL characters, and `..` escapes. Artifact persistence should use a package subdirectory such as `output/`; reject final paths or intermediate components that are symlinks, and keep MJCF and MJB destinations distinct. This keeps returned sensor paths portable and prevents an action from writing outside the generated environment.

Reset must restore `MjData`, task state, controller state, deterministic seed, and any dynamically enabled constraints. If direct qpos assignment represents a grasp or teleport, label it as a task-level abstraction rather than physical contact validation.
