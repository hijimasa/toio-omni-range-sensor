"""Tests for the motor deadband compensation.

Runnable without ROS: `python3 -m pytest src/toio_omni_bringup/test`.

The property that matters is the last one: whatever comes out of the scaling has
to still be at least 8 *after* toio_ros2 truncates it with int(). A scaled
7.9999 is the same standstill as an unscaled 7.
"""

import math

from toio_omni_bringup.motor_deadband import (MIN_MOTOR, motor_command, motor_units,
                                              scale_for_deadband, wheel_speeds)


def test_motor_units_matches_the_driver_arithmetic():
    # 0.05 m/s is the planner's speed cap; toio_ros2 sends 17 for it
    assert motor_command(0.05) == 17
    assert math.isclose(motor_units(0.05), 17.79, abs_tol=0.01)


def test_the_deadband_is_where_it_was_calculated_to_be():
    assert motor_command(0.0224) < MIN_MOTOR
    assert motor_command(0.0225) >= MIN_MOTOR


def test_pure_rotation_at_the_old_limit_was_dead():
    # 1.5 rad/s in place: the reason the cube never turned towards its path
    left, right = wheel_speeds(0.0, 1.5)
    assert motor_command(right) == 7


def test_rotation_is_scaled_until_it_turns():
    v, omega = scale_for_deadband(0.0, 1.5)
    assert v == 0.0
    assert omega > 1.5
    left, right = wheel_speeds(v, omega)
    assert motor_command(right) >= MIN_MOTOR
    assert motor_command(left) <= -MIN_MOTOR


def test_curvature_is_preserved():
    v, omega = 0.004, 0.3
    sv, somega = scale_for_deadband(v, omega)
    assert sv > v and somega > omega
    assert math.isclose(sv / somega, v / omega, rel_tol=1e-9)


def test_commands_above_the_deadband_pass_through_untouched():
    assert scale_for_deadband(0.05, 0.0) == (0.05, 0.0)
    assert scale_for_deadband(-0.05, 1.0) == (-0.05, 1.0)


def test_a_stop_stays_a_stop():
    assert scale_for_deadband(0.0, 0.0) == (0.0, 0.0)


def test_numerical_dust_is_not_boosted_into_a_lurch():
    assert scale_for_deadband(1e-6, 0.0) == (0.0, 0.0)
    assert scale_for_deadband(0.0, 1e-5) == (0.0, 0.0)


def test_sign_is_preserved():
    v, omega = scale_for_deadband(-0.005, 0.0)
    assert v < 0
    assert motor_command(v) <= -MIN_MOTOR


def test_scaled_commands_survive_the_drivers_truncation():
    # sweep the whole space the planner can ask for
    for i in range(-40, 41):
        for j in range(-40, 41):
            v, omega = i * 0.0025, j * 0.1
            sv, so = scale_for_deadband(v, omega)
            if (sv, so) == (0.0, 0.0):
                continue
            left, right = wheel_speeds(sv, so)
            peak = max(abs(motor_command(left)), abs(motor_command(right)))
            assert peak >= MIN_MOTOR, \
                'v=%.4f omega=%.2f scaled to %d motor units' % (v, omega, peak)
