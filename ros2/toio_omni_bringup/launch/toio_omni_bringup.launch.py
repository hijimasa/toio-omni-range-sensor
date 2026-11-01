"""Cube and range sensor together, with the scan placed on the mat.

  ros2 launch toio_omni_bringup toio_omni_bringup.launch.py
  ros2 launch toio_omni_bringup toio_omni_bringup.launch.py \
      cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF

The TF chain is map -> center -> omni_link: toio_ros2 puts the cube on the mat
from the Position ID, and the static transform here says where the mirror sits
on the cube. That is the whole integration -- the two drivers do not know about
each other, they only meet in TF.

Two arguments deserve a look before trusting what RViz draws:

* `sensor_yaw` is the rotation of the sensor's 0 degrees against the cube's
  forward axis, and it is not known to be zero. Check it the way the mirror's
  `ccw` was checked: put an object at a known bearing, drive the cube through a
  known rotation, and see whether the scan follows. A wrong yaw produces a scan
  that looks entirely reasonable and is wrong by a constant.
* `sensor_z` is a placeholder (the top of the cube body). It only matters for
  drawing and for a 3D costmap, but it is not measured.

`remap_cube_tf` is for later. The Position ID gives an absolute pose, so once a
localiser also wants to publish into `map` the two would fight; turning it on
moves the cube's transform to /tf_gt, where it becomes ground truth to score the
localiser against instead of a competitor. Off by default, because until that
node exists the cube's transform is the only thing placing the scan.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    omni_range_dir = get_package_share_directory('toio_omni_range')
    bringup_dir = get_package_share_directory('toio_omni_bringup')

    # The A3 developer mat folded in half, which is what this workspace runs
    # on. toio_ros2's own files describe unfolded mats; using one of those here
    # scales every distance by 0.707 and sends goals past the crease.
    default_params_file = os.path.join(
        bringup_dir, 'params', 'toio_a3_folded_left_params.yaml')
    # Generated from the mirror calibration by tools/gen_firmware_calib.py.
    calib_params_file = os.path.join(
        omni_range_dir, 'params', 'omni_range_calib.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'toio_omni.rviz')
    collision_params_file = os.path.join(bringup_dir, 'params', 'collision_monitor.yaml')

    namespace = LaunchConfiguration('namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')
    params_file = LaunchConfiguration('params_file')
    cube_id = LaunchConfiguration('cube_id')
    device_address = LaunchConfiguration('device_address')
    ble_adapter = LaunchConfiguration('ble_adapter')
    sensor_x = LaunchConfiguration('sensor_x')
    sensor_y = LaunchConfiguration('sensor_y')
    sensor_z = LaunchConfiguration('sensor_z')
    sensor_yaw = LaunchConfiguration('sensor_yaw')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher')
    remap_cube_tf = LaunchConfiguration('remap_cube_tf')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')

    declare_cmds = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the nodes and topics (e.g. toio1)'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='',
            description='TF frame prefix of this robot (e.g. "toio1/")'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Mat parameters for toio_ros2'),
        DeclareLaunchArgument(
            'cube_id', default_value='',
            description='Connect only to the cube whose BLE local name contains this'),
        DeclareLaunchArgument(
            'device_address', default_value='',
            description='BLE address of the range sensor. Setting it skips the '
                        'discovery scan, which is what disturbs the cube on the '
                        'same adapter'),
        DeclareLaunchArgument(
            'ble_adapter', default_value='',
            description='BlueZ adapter for the range sensor, e.g. hci1. Empty '
                        'uses the default, which the cube is also on'),
        DeclareLaunchArgument(
            'sensor_x', default_value='0.0',
            description='Mirror axis forward of the cube centre, metres'),
        DeclareLaunchArgument(
            'sensor_y', default_value='0.0',
            description='Mirror axis left of the cube centre, metres'),
        DeclareLaunchArgument(
            'sensor_z', default_value='0.0256',
            description='Mirror height above the floor, metres. Placeholder: '
                        'this is the top of the cube body, not a measurement'),
        DeclareLaunchArgument(
            'sensor_yaw', default_value='0.0',
            description='Sensor 0 deg against the cube forward axis, radians. '
                        'Verify by moving the cube, not from a still image'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Use RViz2 if true'),
        DeclareLaunchArgument(
            'use_robot_state_publisher', default_value='false',
            description='Publish the cube model. Needs toio_description in the '
                        'workspace'),
        DeclareLaunchArgument(
            'remap_cube_tf', default_value='false',
            description='Send the cube transform to /tf_gt instead of /tf, so a '
                        'localiser can own map without a conflict'),
        DeclareLaunchArgument(
            'use_collision_monitor', default_value='false',
            description='Gate cmd_vel on the scan. Off by default only because '
                        'turning it on moves the teleop input to cmd_vel_raw, '
                        'which looks like an unresponsive robot if unexpected. '
                        'The nav2 launch turns it on'),
    ]

    # toio_ros2 subscribes to goal_pose and drives there with the cube's own
    # motor_control_target, straight over BLE. That command reaches the motors
    # without passing through cmd_vel, so nothing downstream -- not the
    # collision monitor, not a costmap -- can see or stop it. Left on the
    # standard topic it also races nav2, which listens to the same one: RViz's
    # 2D Goal Pose would make the cube drive itself while nav2 planned in vain.
    # So the cube's built-in motion gets its own name, and /goal_pose belongs to
    # whatever is actually navigating.
    cube_node_kwargs = dict(
        package='toio_ros2',
        executable='toio_ros2_node',
        name='toio_ros2_node',
        namespace=namespace,
        parameters=[params_file,
                    {'cube_id': cube_id, 'frame_prefix': frame_prefix}],
        output='screen')
    cube_remaps = [('goal_pose', 'cube_goal_pose')]

    # One node, two remapping variants: launch cannot make a remapping itself
    # conditional, so the condition has to select between whole actions.
    toio_ros2_node = Node(
        condition=UnlessCondition(remap_cube_tf),
        remappings=cube_remaps,
        **cube_node_kwargs)

    toio_ros2_node_gt = Node(
        condition=IfCondition(remap_cube_tf),
        remappings=cube_remaps + [('/tf', '/tf_gt'), ('/tf_static', '/tf_static_gt')],
        **cube_node_kwargs)

    omni_range_node = Node(
        package='toio_omni_range',
        executable='omni_range_node',
        name='omni_range_node',
        namespace=namespace,
        parameters=[calib_params_file,
                    {'device_address': device_address,
                     'ble_adapter': ble_adapter,
                     'frame_id': [frame_prefix, 'omni_link']}],
        output='screen')

    sensor_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='omni_link_tf',
        namespace=namespace,
        arguments=['--x', sensor_x, '--y', sensor_y, '--z', sensor_z,
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', sensor_yaw,
                   '--frame-id', [frame_prefix, 'center'],
                   '--child-frame-id', [frame_prefix, 'omni_link']],
        output='screen')

    # Resolved lazily so the launch file still works when toio_description is
    # not in the workspace.
    toio_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('toio_description'),
            'launch', 'robot_description.launch.py'])),
        condition=IfCondition(use_robot_state_publisher),
        launch_arguments={'namespace': namespace,
                          'frame_prefix': frame_prefix}.items())

    # collision_monitor is a lifecycle node and stays unconfigured until
    # something drives it through the transitions, hence the manager.
    collision_monitor_node = Node(
        condition=IfCondition(use_collision_monitor),
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        namespace=namespace,
        parameters=[collision_params_file,
                    {'base_frame_id': [frame_prefix, 'center']}],
        output='screen')

    collision_lifecycle_node = Node(
        condition=IfCondition(use_collision_monitor),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        namespace=namespace,
        parameters=[collision_params_file],
        output='screen')

    # Always on, and harmless when nothing needs it: commands already above the
    # deadband pass through untouched. Without it a planner's turn-in-place
    # command reaches the cube as a motor value of 7 and nothing happens.
    deadband_node = Node(
        package='toio_omni_bringup',
        executable='cmd_vel_deadband',
        name='cmd_vel_deadband',
        namespace=namespace,
        output='screen')

    rviz2_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    ld = LaunchDescription()
    for cmd in declare_cmds:
        ld.add_action(cmd)
    ld.add_action(toio_ros2_node)
    ld.add_action(toio_ros2_node_gt)
    ld.add_action(omni_range_node)
    ld.add_action(sensor_tf_node)
    ld.add_action(toio_description_launch)
    ld.add_action(deadband_node)
    ld.add_action(collision_monitor_node)
    ld.add_action(collision_lifecycle_node)
    ld.add_action(rviz2_node)
    return ld
