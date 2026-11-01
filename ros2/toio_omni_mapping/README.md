[English](README.md) | [日本語](README.ja.md)

# toio_omni_mapping

Drives around the mat and builds a map automatically. The pose comes from the Position ID, so
**this is not SLAM but "occupancy grid building with known poses"**.

```bash
ros2 launch toio_omni_mapping mapping.launch.py \
    cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF
# once "coverage complete" appears (before leaving the container!)
ros2 run nav2_map_server map_saver_cli \
    -f src/toio-omni-range-sensor/ros2/toio_omni_bringup/map/arena \
    --ros-args -p save_map_timeout:=5.0
```

**Save inside the mounted `src/`.** The container's workspace root is not mounted
and the container is started with `--rm`, so a map written to the working directory
vanishes the moment you press Ctrl-C.

In the RViz Map display you can watch the map grow as the cube drives.

## Why not slam_toolbox

The Position ID is absolute and does not drift. **Scan matching and loop closure, the hard
parts of SLAM, are not only unnecessary; adding them makes the result worse.** A matcher
handed a 23 cm, 72-beam scan can do little, and its output becomes a **correction to a pose
that was already correct**. That all its parameters are in meter scale is a separate problem
that also bites.

## Physical setup

**Without walls nothing shows up.** The outline drawn by `make_mat_map.py` is a virtual wall
(it is only a planning wall, because the Position ID is lost off the mat) and the sensor
cannot see it. On a bare mat every ray is `inf` and the map is "free space only".

**Walls must be a saturated color.** This sensor discriminates by the saturation difference
from the floor (measured at 6 to 10 times the floor). A white wall on a white floor does not
show up. Blue tones are reliable.

**Height** must exceed the mirror (about 2.6 cm or more above the floor).

### Visibility geometry

For an A4 mat 0.297 × 0.210 with `range_max` = 0.1108 m:

| Position | Nearest wall | |
|---|---|---|
| Center | 0.105 m to the long edge | **Barely visible** (5 mm margin, worst-case resolution 0.067 cm/px)|
| Center | 0.1485 m to the short edge | Not visible |

With walls along the perimeter, "at least one wall is visible from anywhere" holds. However,
**at the center only two parallel walls are visible, and the direction along the wall is
unconstrained**. If you use this to evaluate localization:

- **Place one or two asymmetric pillars** (this also breaks the rotational symmetry and makes global initialization possible)
- or **shrink to about 20×20 cm** (multiple walls visible everywhere)

## Stop, then measure

**"Stationary" is decided by displacement, not velocity.** One Position ID unit is about
1.4 mm and the notification interval is about 40 ms, so differentiating a one-unit flicker
gives 0.035 m/s. With a velocity threshold, **every scan is discarded as "moving" even when
the cube is completely still**. Within 3 mm over a 0.3 s window counts as stationary.

Behavior verified offline:

| | Fraction judged stationary |
|---|---|
| Still, ±1 unit flicker | 100% |
| Still, ±2 units | 44% (3 scans per 1.5 s stop, enough in practice)|
| Driving at 0.03 m/s | 2% |
| Creeping at 0.01 m/s | 62% (at this speed the BLE-latency error of 0.5 mm is below a cell, harmless)|

**When the cube stops, Position ID notifications stop too.** Silence is treated as
"stationary", not "unknown" (`pose_timeout`). Without this the last velocity estimate
freezes and nothing is ever integrated again.

### TF is looked up at latest, not at the stamp

A scan's stamp is the BLE arrival time, tens of ms later than the camera frame, and
usually **newer than the latest TF**. Looking up by stamp asks tf2 to extrapolate into the
future, and every scan is dropped.

Since we integrate only while stopped, the latest TF equals the TF at capture time.
`TransformListener` uses `spin_thread=True` (otherwise the buffer is not updated while
waiting and the timeout is meaningless).

`stationary_only` (default true) **integrates only scans taken while stopped**. The BLE
latency is eliminated rather than modeled. The coverage drive stops at every point anyway,
so nothing is lost. When building a map with teleop too, stop and wait instead of keeping
the cube moving.

## First: do the mat params match the physical mat

`toio_ros2` converts the Position ID to meters with `field_width_meter` /
`field_height_meter`. **If these are off, the pose, the map and the driving range are all
off.** That is why this package's launch reads the mat dimensions from the params, and
**changing the numbers on the launch side alone does not fix it** (the conversion is done
by `toio_ros2`).

Check: move the cube exactly 10 cm with a ruler and look at the difference in `/toio/pose`.

```bash
ros2 topic echo /toio/pose --field pose.position
```

If it is not 10 cm, the params are wrong. One Position ID unit is about 1.4 mm, so on A4
(0.21 × 0.297 m) the ID span should be roughly 149 × 210 units. The A4 params bundled with
`toio_ros2` assign 0.297 × 0.210 m to 304 × 216 units, and **the two are incompatible**
(304 units ≈ 0.43 m). Make a params file that matches your mat and pass it with
`params_file:=`.

## Coverage drive

It just streams a back-and-forth pattern to `cube_goal_pose`. This goes down to the cube's
built-in `motor_control_target`, so **the mat closes the loop in hardware and nav2 is not
needed** (it could not be used anyway, since the map does not exist yet).

**These commands do not pass through `cmd_vel`, so the collision_monitor cannot stop them.**
`motor_control_target` is a closed loop on the Position ID and keeps pushing until it
reaches the target or times out, shoving anything in the way aside. So this node does the
monitoring itself. If anything within ±25 degrees of the heading is closer than
`stop_distance` (default 5 cm), it publishes `cmd_vel` to take the motors back (**a normal
motor command overrides the target motion**), backs up a little and gives up on that
waypoint.

The `settle_time` wait after backing up is there because `toio_ros2` sends a single stop
command when `cmd_vel` goes quiet. If the next target is issued before that arrives, the stop
takes effect later and cancels the target motion.

**Since the sensor is omnidirectional, no in-place rotation is needed at all.** Coverage is
done with translation only.

| Parameter | Default | |
|---|---|---|
| `row_spacing` | 0.05 | Spacing between scan lines |
| `inset` | 0.035 | Margin from the mat edge. The Position ID cannot be read at the edge |
| `stop_distance` | 0.05 | Give up on the waypoint if something is within this distance |
| `dwell_time` | 1.5 | Stop time at each point. Pairs with `stationary_only` |
| `goal_timeout` | 25.0 | Move on even if the goal is not reached |

The arrival check watches `toio/pose`. `toio_ros2` logs the motor response but does not
publish it on a topic, so there is nothing to subscribe to.

**`inset` cannot be smaller than `stop_distance`.** A waypoint closer than that to a wall is
inside the abort radius and can never be reached in principle (it is abandoned on every
approach). When the walls are at the mat edge, the row ends are exactly that, so `_plan()`
automatically widens it to `stop_distance + edge_clearance` and logs it. The sensor reaches
23 cm, so the wall is still visible from farther away.

Start waits for the first `/toio/pose`. **Goals sent before the connection are dropped**, so
with a fixed wait the first one is lost on a day when the BLE scan takes 20 s.

`use_coverage_drive:=false` switches to teleop map building.

## Using the resulting map with nav2 / AMCL

### The sensor cannot see the mat boundary

This matters most. The mat is a sheet of paper on the floor, with no saturation step at its
edge. **To the sensor, outside the mat is the same "floor = free"**, so the measured map
extends as free space beyond the mat (measured up to 4.3 cm on each side, 144 cm² worth).

Run nav2 on that map and **the planner routes paths off the mat**. The moment the cube
leaves, it loses the Position ID and the pose freezes. Worse than a collision, and a failure
mode nobody reports.

```bash
python3 tools/clip_map_to_mat.py --map map/arena.yaml    # on the toio_omni_bringup side
```

This outputs `arena_clipped.yaml` with everything outside the mat rectangle turned into
wall. Measurements inside the mat are left as is. **This is a map for planning, not for
localization** (it asserts walls the sensor never observes, so use the unmodified one for
AMCL).

**`track_unknown_space: true`** (already set in `nav2_params.yaml`). The measured map keeps
unknown cells (behind walls, corners that could not be reached). With `false` they are
treated as free.

And when using it to evaluate localization, note that **the map was built with the same
sensor**. The sensor's systematic error enters both the map and the observations, so AMCL's
error comes out optimistic in absolute terms. For comparing methods against each other the
conditions are identical, so this is not a problem.

## Verification status

One full run on real hardware has confirmed **91% of the grid known, zero
`without a transform`** (A3 folded mat, 12 waypoints, about 1 minute).

`grid.py` (ray casting and log-odds) is unit-tested without ROS. It checks endpoint
handling, that a wall seen from both sides lands in the same cell, and that a miss ray does
not wrap around to the opposite side. The stationary-check threshold was also chosen by an
offline sweep with Position ID quantization noise added (table above).

Not verified: whether the finished map **matches the actual wall positions**. Every defect
so far has been of the "plausibly wrong" kind, so view the map with that assumption too.
