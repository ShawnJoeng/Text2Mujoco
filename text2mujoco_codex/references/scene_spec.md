# Scene Specification

`scene_spec.json` is the stable boundary between natural-language interpretation and MuJoCo-specific code. It uses SI meters, Z-up poses, full geometry dimensions, and required `xyzw` quaternions. The generator converts these values to MJCF conventions; it does not perform unit or axis conversion.

## Required Top-Level Fields

```json
{
  "schema_version": "1.0",
  "backend": "mujoco",
  "source_prompt": "Place a cube in an open box after pressing a button.",
  "assumptions": [],
  "open_questions": [],
  "scene": {
    "name": "button_cube_box",
    "runtime": {
      "mujoco_version": "3.2.7",
      "mode": "python",
      "headless": true,
      "gl_backend": "auto"
    },
    "world": {
      "units": "m",
      "up_axis": "Z",
      "gravity": [0, 0, -9.81],
      "ground": true,
      "seed": 0
    }
  },
  "assets": [],
  "interaction_points": [],
  "sensors": [],
  "task": {
    "goal": "Complete the requested interaction sequence.",
    "success_conditions": [],
    "failure_conditions": [],
    "reset_policy": "mj_resetData_and_task_state"
  },
  "outputs": {
    "save_mjcf": true,
    "mjcf_path": "./output/model.xml",
    "save_mjb": true,
    "mjb_path": "./output/model.mjb"
  }
}
```

## Assets and Names

Each asset needs a stable `id`, `kind`, `body_name`, pose, and physics object. Primitive and mesh assets also need geometry; includes and robots instead declare their source and exported typed names. MuJoCo names are typed: a body, geom, joint, actuator, site, and camera may share text, but names must be unique within each type.

```json
{
  "id": "red_cube",
  "kind": "primitive",
  "body_name": "red_cube",
  "geometry": {
    "shape": "box",
    "dimensions": [0.1, 0.1, 0.1],
    "geom_name": "red_cube_geom"
  },
  "pose": {
    "position": [0, 0, 0.825],
    "orientation_xyzw": [0, 0, 0, 1]
  },
  "physics": {
    "dynamic": true,
    "joint_name": "cube_free",
    "mass_kg": 0.2,
    "contype": 1,
    "conaffinity": 1,
    "friction": [0.8, 0.01, 0.001]
  }
}
```

Primitive `dimensions` are full extents. For `box`, use `[x, y, z]`; for `cylinder`, x and y are equal diameters and z is height; for `sphere`, all three values are the diameter. An `open_box` uses full outer dimensions, an `interior_bounds` object in body-local coordinates, and exactly named `part_geom_names` for its colliders.

Allowed kinds are `primitive`, `mesh`, `mjcf_include`, and `robot`. Primitive assets must declare shape and dimensions; mesh assets declare a source/mesh path and geom name; includes/robots declare a source path and are loaded according to their own model boundary. An include or robot that exposes interaction targets must declare typed names in `exports`, for example `{"joint": ["arm_joint"], "actuator": ["arm_motor"]}`; runtime compilation must still confirm those exports exist. Do not imply that a URDF can be inserted into arbitrary MJCF without conversion. Resolve paths relative to the generated package or a caller-provided asset root.

`physics.dynamic: true` normally maps to a free joint unless the asset declares another joint. MuJoCo friction is a three-value vector. Use `contype` and `conaffinity` for collision filtering; do not translate a simulator-specific restitution scalar directly.

## Typed Targets and Interactions

An interaction target is an explicit MuJoCo object reference:

```json
{
  "id": "press_start_button",
  "target": {"type": "joint", "name": "button_slide"},
  "marker_site": "press_start_button_marker",
  "affordance": "press",
  "pose": {"position": [0.38, -0.18, 0.83], "orientation_xyzw": [0, 0, 0, 1]},
  "action": {
    "mode": "direct",
    "command": "press_start_button",
    "schema": {"type": "object", "properties": {}, "required": []}
  },
  "preconditions": ["button_state == 'ready'"],
  "success_conditions": ["button_state == 'pressed'"],
  "depends_on": [],
  "effects": ["button_state = 'pressed'"],
  "reset": {"button_state": "ready"}
}
```

Allowed target types are `body`, `geom`, `joint`, `actuator`, `site`, and `camera`. The referenced name must be declared by an asset, sensor, or interaction marker. Every interaction must have a JSON-compatible action schema, observable conditions, dependencies, effects, and reset data.

A `marker_site` must resolve to a site that is actually drawn for the viewer (`group="2"`) and lying flush on a real surface, not a sphere floating in the air — see [interaction_patterns.md](interaction_patterns.md). Its `pose` here and the site's `pos` in MJCF are two copies of one number and drift apart silently; re-derive the spec pose from the MJCF whenever the model moves. A pose that is really the camera position, or a marker over floor the robot never reaches, means the interaction is annotating nothing.

The bundled validator accepts the object-shaped action-schema subset used by the runtime (`type`, `properties`, `required`, scalar bounds, and array item/length constraints) and rejects malformed field definitions. `scene.world.units` must be `m` and `scene.world.up_axis` must be `Z`; the validator does not perform unit or axis conversion. It also rejects credential-like text, email addresses, and machine-local paths in persisted text. External URI references are rejected for asset and output paths; ordinary descriptive URLs may remain in a prompt when they contain no credential-like material.

## Sensors and Outputs

A camera sensor declares `camera_name`, positive `frequency_hz`, and the fields returned by `observe()` or capture, such as `rgb`, `depth`, and `camera_pose`. Other sensors should use a stable typed MuJoCo object name and document units.

`outputs` declares package-relative destinations, not evidence that files exist. `save_mjcf` requires `mjcf_path`; `save_mjb` requires `mjb_path`. Absolute paths and `..` escapes are rejected. Runtime capture directories follow the same package-root boundary. A generated `.mjb` must be reloaded with `MjModel.from_binary_path` before being marked verified.

## Quaternion Conversion

The specification stores `[x, y, z, w]`. MJCF's `quat` attribute expects `[w, x, y, z]`. Normalize, reorder, and test this conversion explicitly. Free-joint qpos uses the same MuJoCo `wxyz` order.

Runtime geometry helpers must apply declared orientations. Container bounds expressed in body-local coordinates must be transformed into world coordinates before evaluating placement. An implementation that supports only axis-aligned containers must state and validate that limitation rather than silently ignoring rotation.
