[English](README.md) | [日本語](README.ja.md)

# toio_omni_bringup

Launches the toio cube (`toio_ros2`) and the omnidirectional range sensor (`toio_omni_range`) together.

**The two drivers know nothing about each other. They meet only in TF.**

```
map ──(toio_ros2: Position ID from the mat)──> center ──(static)──> omni_link
                                                                        │
                                                              scan (LaserScan)
```

## Launching

```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py \
    cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF
```

`device_address` is the sensor, `cube_id` is the cube. **Specify both**
(see "Sharing the BLE adapter" below for why).

| Argument | Default | Meaning |
|---|---|---|
| `device_address` | `''` | BLE address of the sensor. When given, the discovery scan is skipped |
| `ble_adapter` | `''` | BlueZ adapter for the sensor side (`hci1` etc.) |
| `cube_id` / `params_file` | `''` / A4 | Passed straight through to `toio_ros2` |
| `sensor_x/y/z/yaw` | `0,0,0.0256,0` | Position of the mirror as seen from `center` |
| `remap_cube_tf` | `false` | Divert the cube's TF to `/tf_gt` (when adding a localization node) |
| `use_robot_state_publisher` | `false` | Only if `toio_description` is available |

## `goal_pose` is remapped to `cube_goal_pose`

`toio_ros2` subscribes to `goal_pose` and drives straight there with the cube's
built-in `motor_control_target`. **This command goes to the motors directly over
BLE and never passes through `cmd_vel`.** So neither collision_monitor nor the
costmaps have any effect on it.

Left on the standard `/goal_pose`, it competes with nav2: on a 2D Goal Pose in RViz
**the cube drives straight ahead on its own while nav2 spins its wheels**. This is
exactly how avoidance failed to work in practice.

So the built-in target motion gets a dedicated name, and `/goal_pose` is left to
whatever is actually navigating. To use the cube's built-in motion directly:

```bash
ros2 topic pub --once /cube_goal_pose geometry_msgs/PoseStamped ...
```

## Mounting the sensor

`sensor_yaw = 0` (sensor 0 degrees = cube front) is **verified**.

**`sensor_z` is not a measured value** (it is just the cube's top face, 0.0256 m).
It only affects visualization and the 3D costmap, but fix it once measured.

## Sharing the BLE adapter

The cube (toio-py → bleak) and the sensor (bleak) use the same HCI adapter.
A discovery scan destabilizes the connections sharing it, so the first measure
is to specify both the address and the ID and avoid scanning altogether.

**Measure, do not guess.** Procedure:

```bash
# 1. Baseline with the sensor alone
ros2 launch toio_omni_range omni_range.launch.py device_address:=AA:BB:...
ros2 topic hz /scan --window 200
ros2 topic echo /diagnostics          # dropped / reconnects / rate_hz

# 2. The same measurement under bringup
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py \
    device_address:=AA:BB:... cube_id:=a1b
```

Numbers to watch:

| Source | Value | Meaning |
|---|---|---|
| `/diagnostics` | `rate_hz` | Has it dropped from 5 Hz (`scan_period` default 200ms)? |
| `/diagnostics` | `dropped` | Missing notifications. Counted from gaps in seq |
| `/diagnostics` | `reconnects` | Number of reconnects on the sensor side |
| `toio_ros2` log | `toio is disconnected. reconnecting...` | Disconnect on the cube side |

If standalone and shared show no difference, you are done. If they differ, in this order:

1. Eliminate the discovery scan by specifying addresses (if not already)
2. Raise `scan_period` to reduce the sensor's bandwidth (`ble_client.py --cmd scan-period`)
3. Add a USB dongle and separate them physically with `ble_adapter:=hci1`

**Merging into one process is the last resort**, and it does not require folding
the drivers into one (i.e. forking `toio_ros2`).

## The avoidance layer (`use_collision_monitor`, default false)

`nav2_collision_monitor` is inserted in front of `cmd_vel`. It is not hand-rolled
because scan coordinate transforms, stale-data timeouts, and deceleration instead
of an abrupt stop are already implemented correctly there.

```bash
ros2 launch toio_omni_bringup toio_omni_bringup.launch.py \
    use_collision_monitor:=true device_address:=AA:BB:...

# Input moves to cmd_vel_raw. Unless speed/turn are lowered (default 0.5 m/s) it topples, mirror and all
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw \
    -p speed:=0.03 -p turn:=1.8
ros2 topic echo /collision_monitor_state
```

The default is false because enabling it **moves the teleop input to `cmd_vel_raw`**.
Enable it unknowingly and it looks like "the robot stopped responding".
In the nav2 launch it is always enabled.

### The radius is set by the sensor, not the robot

| | Value | Origin |
|---|---|---|
| Blind zone | ~3.6 cm | `range_min`. Anything inside might as well not exist |
| footprint radius | 2.3 cm | Circumscribed circle of the 32 mm cube. Matches `robot_radius` |
| `time_before_collision` | 2.0 s | 6 cm of lookahead at 0.03 m/s. Just outside the blind zone |
| `simulation_time_step` | 0.02 s | 0.6 mm/step at top speed. Does not step over a 5 mm object |
| `min_points` | 2 | Small objects return only a few directions (the lesson from step 3) |

### An approach polygon, not a fixed stop circle

The easy thing to write is a stop polygon with a fixed radius, but **in a 20 cm
arena that is wrong**. A wall is always within a few centimetres, so the stop circle
fires merely from drifting toward an edge, and since stop ignores the direction of
the command, **you cannot even back away to escape**.

`approach` projects the footprint along the commanded velocity and intervenes only
when that motion would actually collide. Motion away from the wall passes through.

The visible band is only 7 cm, from 4.1 to 11.1 cm. At 5 Hz, crossing that band at
5 cm/s takes 1.4 seconds = 7 scans. **This is what sets the top speed this avoidance
layer can deliver.** To go faster, lower the sensor's `scan_period`.

All these numbers are starting points. Drive at an obstacle and tune them while
watching `collision_monitor_state`.

## nav2

```bash
ros2 launch toio_omni_bringup toio_omni_nav2.launch.py \
    cube_id:=a1b device_address:=AA:BB:...
```

Give goals with **2D Goal Pose** in RViz. No `initialpose` is needed
(the mat's Position ID is an absolute position, so AMCL itself is not needed).

The command path:

```
controller_server / behavior_server ──> cmd_vel_raw
                                            │
                                     collision_monitor
                                            │  cmd_vel_gated
                                     cmd_vel_deadband
                                            │
                                         cmd_vel ──> toio_ros2 ──> cube
```

### The motor deadband

The conversion in `toio_ros2` is `motor_speed = int(355.7 × wheel speed)`. **The cube
does not turn its wheels for values below 8.**

| Command | Wheel speed | motor_speed | |
|---|---|---|---|
| In-place rotation at 1.5 rad/s | 0.0200 m/s | **7** | Does not turn |
| Straight at 0.05 m/s | 0.0500 m/s | 17 | Turns |

**Rotation dies entirely.** DWB first turns toward the path, so it freezes there and
15 seconds later reports `Failed to make progress`. No error appears anywhere.

`cmd_vel_deadband` scales **both wheels by the same ratio** until the faster wheel
reaches 8 (curvature is preserved). Commands above the deadband pass through untouched.

It sits **after collision_monitor** because `approach` throttles the command and
would otherwise push it back under the deadband.

The effective minimum in-place rotation is **1.69 rad/s**. `max_vel_theta` is set
slightly above that, at 2.0. It cannot be lowered: if most of the range is deadband,
"the planner asked for a turn that never actually happens" becomes the normal state;
and at 3.0 the top-heavy robot toppled.

### Top-heavy: speed is set by stability

A camera, holder and mirror sit on top of a 32 mm cube, so the centre of mass is high.
It topples on abrupt starts and stops and on in-place rotation, so **the speed limit is
set by not falling over, not by the sensor.** There are three paths that produce speed,
and lowering only one of them is pointless.

| Path | Setting | Value |
|---|---|---|
| nav2's `cmd_vel` | `max_vel_x` / `acc_lim_*` in `nav2_params.yaml` | 0.03 m/s, 0.15 m/s², 2.0 rad/s, 8 rad/s² |
| The cube's built-in target motion (`cube_goal_pose`, used by mapping) | launch argument `goal_max_speed` | 15 (motor value. The `toio_ros2` default of 30 ≈ 8 cm/s topples) |
| teleop | `speed` / `turn` of `teleop_twist_keyboard` | 0.03 m/s, 1.8 rad/s |
| nav2 recovery (BackUp) | `backup_dist` / `backup_speed` in `behavior_trees/navigate_to_pose_toio.xml` | 0.03 m, 0.025 m/s |

**The fourth path is easy to miss.** nav2's default Behavior Tree backs up **0.30 m at
0.15 m/s** during recovery. That is five times the speed limit over the length of a whole
mat, and it is what "it suddenly reversed fast" really was. Distance and speed are ports on
the BT node, so they cannot be changed from `nav2_params.yaml`, and BackUp in Jazzy has no
acceleration or minimum-speed parameters either. So the BT is copied into the package with
only BackUp changed to 3 cm and 0.025 m/s, and the launch points at it via
`default_nav_to_pose_bt_xml`. collision_monitor watches reverse motion too, but the rear is
only a target of the `approach` projection; it does not limit the speed itself.

What cannot be lowered is the deadband: motor values below 8 do not move. Straight at
0.0225 m/s and in-place rotation at 1.69 rad/s are "the gentlest possible start"; below that
the choice is 0 or 8. collision_monitor's `time_before_collision` is tuned so that its product
with `max_vel_x` is 6 cm (change them together when changing the speed).

velocity_smoother is not included. `toio_ros2` already limits BLE writes to 20 Hz and drops
duplicates, and deceleration is already ramped on the collision_monitor side. A third
shaper would add nothing.

Frames:

```
map ──(static identity)──> odom ──(toio_odom_bridge)──> center ──> omni_link
                                                          ▲
                                        toio_ros2's TF goes to /tf_gt
```

`toio_ros2`'s `map → center` is an absolute position and correct, but nav2 requires the
`map → odom → base` shape. So `remap_cube_tf:=true` diverts the cube's TF to `/tf_gt`,
and `toio_odom_bridge` rebuilds the chain from the same `toio/pose`. **Nothing is
fabricated. The same thing is written in a different shape.** There is no renaming to
`base_link`; `center` is kept as the base frame as is.

`toio_odom_bridge` exists for two reasons:

- **When the cube stops, Position ID notifications stop too.** Unless TF is re-published
  on a timer rather than on arrival, nav2 halts on a transform timeout.
- **DWB uses the odom twist as the current velocity.** Differentiating a position
  quantized to 1 mm is mostly noise, so an EMA is applied and it falls to zero when
  updates stop.

### The map is a generated artifact

```bash
python3 tools/make_mat_map.py                       # A4
python3 tools/make_mat_map.py --params ../toio_ros2/params/toio_a3_play_mat_params.yaml
```

The mat dimensions exist only in the `toio_ros2` params. Drawing it by hand creates a
third copy that will eventually diverge. **Regenerate the map whenever `params_file` changes.**

The border is deliberately occupied: off the mat the cube loses the Position ID and is no
longer localized. For planning purposes that really is a wall.

### Defaults changed for a 32 mm robot

nav2's defaults are all for a 50 cm robot inside a building. Only the ones that are not
merely conservative but **wrong at this scale** have been changed.

| Parameter | Default | Here | Reason |
|---|---|---|---|
| `required_movement_radius` | 0.5 | 0.008 | You cannot travel 0.5 m on a 0.297 m mat. Every navigation ends up stuck |
| costmap `resolution` | 0.05 | 0.005 | Cells coarser than the robot |
| `inflation_radius` | 0.55 | 0.035 | Inflation larger than the whole mat |
| `max_vel_x` | 0.26 | 0.03 | The limit for not toppling when top-heavy (above). The sensor alone could keep up at 0.05 |
| `acc_lim_x` / `acc_lim_theta` | 2.5 / 3.2 | 0.15 / 8.0 | Abrupt starts and stops topple it |
| `max_vel_theta` | 1.0 | 2.0 | Slightly above the 1.69 rad/s deadband. Toppled at 3.0 |
| `xy_goal_tolerance` | 0.25 | 0.008 | 1/8 of the mat's length is no tolerance |
| `Oscillation.*` | 0.05 | 0.02 | Normal motion gets judged as oscillation |

The subtle point is that **`obstacle_max_range` is set smaller than `range_max`**.
With `inf_is_valid: True`, nav2 replaces inf with `range_max`, so setting
`obstacle_max_range` equal to `range_max` **lines the horizon with phantom obstacles**.
The values are 0.225 (marking) < 0.233 (range_max) < 0.240 (raytrace).
This way "directions with nothing in them" are properly used for clearing.

### When it does not move

**If `Robot to stop due to invalid source` appears, that is the answer.**
No scan is arriving, so collision_monitor is holding the robot. nav2 is fine.

```bash
ros2 topic hz /scan                  # No output means the sensor is not connected
ros2 topic echo /collision_monitor_state
```

To try nav2 alone without the sensor:

```bash
ros2 launch toio_omni_bringup toio_omni_nav2.launch.py use_collision_monitor:=false
```

The controller then publishes directly to `cmd_vel`. **If it moves, the problem is on
the sensor side.** Naturally there is no avoidance in this mode, so switch back once done.

Other cases:

| Symptom | Where to look |
|---|---|
| `/plan` appears but `/cmd_vel` is empty | collision_monitor (above) |
| `/plan` does not appear either | planner, global_costmap, or TF |
| `Failed to make progress` | The result of 15 seconds without motion. The cause is elsewhere |
| Dies immediately with `parameter ... is of type {integer}` | The costmap `width`/`height` are integers |

## About `remap_cube_tf`

If `toio_ros2` had a `publish_tf` parameter, this remap would be unnecessary.
It is generally useful to anyone using an external localizer, so it would make a
good small PR upstream.
