# Present so pytest puts the package directory on sys.path. colcon runs the
# tests from here and does not need it, but `pytest ros2/toio_omni_range` from
# the repository root does -- and that is how the protocol tests get run on a
# machine with no ROS installed.
