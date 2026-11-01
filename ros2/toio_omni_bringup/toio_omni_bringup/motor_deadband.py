"""The cube's motor deadband, and what a Twist has to be scaled to to clear it.

toio_ros2 turns a Twist into the cube's own motor units with

    motor = int((rpm / max_rpm) * max_input_speed),  rpm = v_wheel / (2*pi*r) * 60

which comes out at about 355.7 * v_wheel. The cube does not turn a wheel for
values below 8, so any command whose faster wheel lands under that produces no
motion at all -- silently, with no error anywhere.

That bites hardest in rotation, because the lever arm is half the wheel base:

    max_vel_theta = 1.5 rad/s  ->  wheel 0.0200 m/s  ->  motor 7  ->  nothing

A planner that turns towards its path before driving therefore sits still, the
odometry keeps reporting zero, and the progress checker eventually gives up. The
fix is to scale the whole command -- both wheels by the same factor, so the turn
radius is preserved -- until the faster wheel clears the deadband.

No ROS here so the arithmetic can be tested without a workspace.
"""

import math

# https://toio.github.io/toio-spec/en/docs/hardware_shape and ble_motor,
# matching the constants toio_ros2 uses.
WHEEL_BASE = 0.0266
WHEEL_RADIUS = 0.00625
MAX_RPM = 494.0
MAX_INPUT_SPEED = 115.0
# The smallest value the cube actually turns a wheel for.
MIN_MOTOR = 8.0


def motor_units(v_wheel):
    """Wheel speed in m/s as the cube's motor value, before truncation."""
    rpm = (v_wheel / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0
    return (rpm / MAX_RPM) * MAX_INPUT_SPEED


def motor_command(v_wheel):
    """Give the truncated integer toio_ros2 will actually send."""
    return int(motor_units(v_wheel))


def wheel_speeds(v, omega, wheel_base=WHEEL_BASE):
    """Left and right wheel speeds in m/s for a body twist."""
    half = omega * wheel_base / 2.0
    return v - half, v + half


def scale_for_deadband(v, omega, min_motor=MIN_MOTOR, zero_threshold=0.5,
                       wheel_base=WHEEL_BASE):
    """Scale a twist until its faster wheel clears the deadband.

    Both components are scaled by the same factor, so the path curvature the
    planner asked for is unchanged -- only the speed along it goes up.

    A command whose peak is below `zero_threshold` motor units is treated as a
    stop rather than scaled: it is numerical dust, and boosting it to the
    deadband would turn a request to hold still into a lurch.
    """
    left, right = wheel_speeds(v, omega, wheel_base)
    peak = max(abs(motor_units(left)), abs(motor_units(right)))
    if peak < zero_threshold:
        return 0.0, 0.0
    if peak >= min_motor:
        return v, omega
    # The epsilon survives toio_ros2's int(), which would turn a scaled 7.9999
    # back into 7 and leave the robot exactly as stuck as before.
    k = (min_motor + 1e-6) / peak
    return v * k, omega * k
