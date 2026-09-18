# dora-openarm-mujoco

MuJoCo simulation node for the [OpenArm](https://github.com/enactic/openarm_mujoco) bimanual robot, designed to run inside a [dora-rs](https://github.com/dora-rs/dora) dataflow.

It replaces the physical follower arms and cameras: it accepts joint-position commands, publishes canonical arm position/state snapshots on request, and renders JPEG camera frames.

## Installation

```bash
uv sync
```

## Quick start

A self-contained dummy dataflow is included for testing without real hardware.
It wires a dummy leader node to the MuJoCo sim and records to disk.

```bash
uv run dora build dataflow-dummy.yaml --uv
uv run dora run dataflow-dummy.yaml
```

![Example](media/example.png)

## Dataflow configuration

### Minimal (headless and position forwarding only no cameras)

```yaml
- id: openarm-mujoco
  build: pip install -e .
  path: dora-openarm-mujoco
  inputs:
    move_position_right: leader/follower_position_right
    move_position_left:  leader/follower_position_left
    request_position: dora/timer/millis/4
  outputs:
    - status
    - position_right
    - position_left
```

### Full (interactive viewer + all cameras, with contacts, can be used for vr teleoperation)

```yaml
- id: openarm-mujoco
  build: pip install -e .
  path: dora-openarm-mujoco
  args: "--viewer --render --enable-collision --ctrl --keyframe home"
  inputs:
    move_position_right: leader/follower_position_right
    move_position_left:  leader/follower_position_left
    request_state: dora/timer/millis/4
  outputs:
    - status
    - state_right
    - state_left
    - camera_wrist_right
    - camera_wrist_left
    - camera_head_left
    - camera_head_right
    - camera_ceiling
```

## Inputs

| ID | Type | Description |
|----|------|-------------|
| `move_position_right` | `float32[8]` or `struct{qpos: float32[8]}` | Target joint positions for the right arm: joints 1-7 then the gripper. Legacy `new_position` structs are also accepted. |
| `move_position_left` | `float32[8]` or `struct{qpos: float32[8]}` | Same layout for the left arm. |
| `request_position` | any | Sample both arms and publish only `position_*`. The payload is ignored; `observation_timestamp` records the snapshot time. |
| `request_state` | any | Sample both arms and publish only `state_*`. The payload is ignored; `observation_timestamp` records the snapshot time. |
| `pose_right` | `float32[7]` | VR controller pose `[x, y, z, qw, qx, qy, qz]`, expressed in the `--origin-frame` frame (default: the scene's `arm_origin` site). Used only with `--debug-frames`. |
| `pose_left` | `float32[7]` | Same for the left controller. |
| `button_x` | `bool[1]` | X button state. Edge-triggered: on press every scene joint on non-arm bodies (freejoint objects plus fixtures like drawers/doors) snaps back to the `--keyframe` pose; with `--randomize-objects` the freejoint objects land at a randomized pose instead. The button must be released to re-arm. |

## Outputs

| ID | Type | Description |
|----|------|-------------|
| `status` | `string[1]` | `ready`, published once on startup. |
| `position_right`, `position_left` | `struct{qpos: float32[8]}` | Joint positions sampled on each `request_position` event. |
| `state_right`, `state_left` | struct | Full state sampled on `request_state`: `qpos`, `qvel`, `qtorque` (float32[8]), `tmos`, `trotor` (int32[8]), `motor_status` (string[8]), and `bus` (struct). Torque is MuJoCo generalized actuator force; temperatures are zero placeholders. |
| `camera_wrist_right` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |
| `camera_wrist_left` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |
| `camera_head_left` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |
| `camera_head_right` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |
| `camera_ceiling` | `uint8[N]` | JPEG frame, ~30 Hz. Requires `--render`. |

Camera outputs carry `metadata={"encoding": "jpeg"}`.

State diagnostics are simulation placeholders, not hardware health measurements.
`motor_status` follows qpos order: joints 1-7 then finger_joint1, with `ENABLED`
for existing joints and `SILENT` for missing joints. `bus.carrier` is always
`true` (bool); the cumulative counters `bus_off`, `error_passive`, `error_warning`,
`ack_error`, `tx_overflow`, `rx_overflow`, and `net_down` are always zero (int64).
The simulator does not model CAN communication, motor faults, or temperature.

Arm snapshots are published only on `request_position` or `request_state`,
including before the first position command. Connect the desired request input
to a timer independent of control outputs. `request_position` publishes only
`position_*`; `request_state` publishes only `state_*`, which includes qpos,
velocity, and actuator force.
Startup publishes `ready`; position commands only update the simulation target.
Each request samples both arms under one lock.
After sampling both arms, the node captures one `time.time_ns()` timestamp
before releasing the lock and uses it as `observation_timestamp` for every
output of that request. This is integer Unix wall-clock nanoseconds, not the
request time or MuJoCo simulation time (`data.time`). Request metadata is copied
without modification except for `timestamp`, which is removed from the copy so
Dora supplies each output message timestamp.

To migrate an older dataflow, rename command inputs from `position_*` to
`move_position_*`, replace MuJoCo's `arm_right_observation` and
`arm_left_observation` outputs with `position_*` or `state_*`, and add a
`request_position` or `request_state` input. The old output ports have been removed. Consumers receive
Arrow structs rather than flat arrays. The dummy example connects recorder's
`arm_right_observation` / `arm_left_observation` inputs to `state_right` /
`state_left`; these recorder input names remain unchanged.

## Arguments

Pass these via the `args:` field in the dataflow YAML, or directly on the command line.

| Argument | Default | Description |
|----------|---------|-------------|
| `--xml PATH` | unset | MJCF scene file to load. Overrides `--scene` when set. |
| `--scene NAME` | `cell` | Bundled scene to load when `--xml` is not set. Choices: `cell`, `demo`, `pedestal`, `bimanual`. |
| `--keyframe NAME` | `home` | Keyframe in the MJCF to reset to on startup. |
| `--randomize-objects [RANGE_M]` | off | Randomize freejoint scene objects (e.g. cubes) on startup and on each `button_x` reset: uniform xy offset within ±`RANGE_M` m of the keyframe pose plus a uniform yaw. `z` and articulated fixtures are unchanged. `RANGE_M` defaults to `0.05` when omitted. |
| `--enable-collision` | off | Enable contact/collision detection. Disabled by default to avoid unexpected joint-locking during teleoperation. |
| `--ctrl` | off | Write incoming positions to `data.ctrl` and step the physics (`mj_step`) to simulate actuator control. The default writes directly to `data.qpos` with `mj_forward`. |
| `--viewer` | off | Open the interactive MuJoCo viewer window. Requires a display. |
| `--render` | off | Enable offscreen camera rendering and publish JPEG frames. Leave off if cameras are not needed. |
| `--debug-frames` | off | Draw VR controller poses as coloured arrows in the viewer. Only visible with `--viewer`. |
| `--origin-frame NAME` | `arm_origin` | Frame incoming `pose_right`/`pose_left` are relative to; the overlay composes poses with its live world pose and draws the reference axes there. Pass `world` for raw world-frame poses. Missing frame → warning + world fallback. |
| `--origin-frame-type TYPE` | `site` | MuJoCo object type of `--origin-frame`. Choices: `body`, `site`, `geom`. |

### Environment variables

Every argument's default can also be set via an environment variable named
`DORA_OPENARM_MUJOCO_` + the upper-cased argument name:

| Environment variable | Argument |
|----------------------|----------|
| `DORA_OPENARM_MUJOCO_XML` | `--xml` |
| `DORA_OPENARM_MUJOCO_SCENE` | `--scene` |
| `DORA_OPENARM_MUJOCO_KEYFRAME` | `--keyframe` |
| `DORA_OPENARM_MUJOCO_ENABLE_COLLISION` | `--enable-collision` |
| `DORA_OPENARM_MUJOCO_CTRL` | `--ctrl` |
| `DORA_OPENARM_MUJOCO_VIEWER` | `--viewer` |
| `DORA_OPENARM_MUJOCO_RENDER` | `--render` |
| `DORA_OPENARM_MUJOCO_DEBUG_FRAMES` | `--debug-frames` |
| `DORA_OPENARM_MUJOCO_ORIGIN_FRAME` | `--origin-frame` |
| `DORA_OPENARM_MUJOCO_ORIGIN_FRAME_TYPE` | `--origin-frame-type` |

Boolean flags accept `1`/`0`, `true`/`false`, `yes`/`no` and `on`/`off`.
`DORA_OPENARM_MUJOCO_VIEWER` additionally accepts an FPS number
(e.g. `60`); `true` enables the viewer at the default 30 Hz.

Explicit CLI arguments override the environment. Boolean flags also gain a
`--no-*` form (e.g. `--no-render`, `--no-viewer`) to turn an
environment-enabled option back off. Set the variables via the `env:` field
of the node in the dataflow YAML:

```yaml
- id: openarm-mujoco
  path: dora-openarm-mujoco
  env:
    DORA_OPENARM_MUJOCO_VIEWER: "60"
    DORA_OPENARM_MUJOCO_RENDER: "true"
```

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
