"""Drive the mat and build a map of it, using the Position ID as the pose.

  ros2 launch toio_omni_mapping mapping.launch.py \
      cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF
  # when it says coverage complete:
  ros2 run nav2_map_server map_saver_cli -f arena --ros-args -p save_map_timeout:=5.0

The grid bounds and the drive area both come from the same toio_ros2 mat
parameter file the cube is launched with, read here rather than copied, so the
map cannot end up describing a different mat than the robot is driving on.

The cube's own map -> center transform stays on /tf for this: the mapper needs
the true pose, and there is no localiser to conflict with yet.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('toio_omni_bringup')
    params_file = LaunchConfiguration('params_file').perform(context)
    resolution = float(LaunchConfiguration('resolution').perform(context))
    margin = float(LaunchConfiguration('grid_margin').perform(context))

    # Read, not copied: toio_ros2 converts Position IDs to metres with these
    # same two numbers, so overriding the size here and not there would only
    # move the disagreement somewhere harder to see. If the drive goes to the
    # wrong places, the fix is a mat parameter file that matches the mat, not a
    # different number in this file.
    with open(params_file) as f:
        mat = yaml.safe_load(f)['toio_ros2_node']['ros__parameters']
    width_m = float(mat['field_width_meter'])
    height_m = float(mat['field_height_meter'])
    # toio_ros2 puts the near corner at the origin and grows y downwards.
    mat_bounds = {'min_x': 0.0, 'max_x': width_m, 'min_y': -height_m, 'max_y': 0.0}

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'toio_omni_bringup.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'params_file': params_file,
            'cube_id': LaunchConfiguration('cube_id'),
            'goal_max_speed': LaunchConfiguration('goal_max_speed'),
            'device_address': LaunchConfiguration('device_address'),
            'ble_adapter': LaunchConfiguration('ble_adapter'),
            'sensor_x': LaunchConfiguration('sensor_x'),
            'sensor_y': LaunchConfiguration('sensor_y'),
            'sensor_z': LaunchConfiguration('sensor_z'),
            'sensor_yaw': LaunchConfiguration('sensor_yaw'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'remap_cube_tf': 'false',
            'use_collision_monitor': 'false',
        }.items())

    mapper_node = Node(
        package='toio_omni_mapping',
        executable='gt_mapper',
        name='gt_mapper',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'resolution': resolution,
            'origin_x': -margin,
            'origin_y': -height_m - margin,
            'width_m': width_m + 2 * margin,
            'height_m': height_m + 2 * margin,
        }])

    coverage_node = Node(
        condition=IfCondition(LaunchConfiguration('use_coverage_drive')),
        package='toio_omni_mapping',
        executable='coverage_drive',
        name='coverage_drive',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[dict(mat_bounds,
                         row_spacing=float(
                             LaunchConfiguration('row_spacing').perform(context)))])

    return [bringup_launch, mapper_node, coverage_node]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('toio_omni_bringup'),
        'params', 'toio_a3_folded_left_params.yaml')

    declare_cmds = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Mat parameters. The grid bounds and the drive area are '
                        'derived from this'),
        DeclareLaunchArgument('cube_id', default_value=''),
        DeclareLaunchArgument('goal_max_speed', default_value='15'),
        DeclareLaunchArgument('device_address', default_value=''),
        DeclareLaunchArgument('ble_adapter', default_value=''),
        DeclareLaunchArgument('sensor_x', default_value='0.0'),
        DeclareLaunchArgument('sensor_y', default_value='0.0'),
        DeclareLaunchArgument('sensor_z', default_value='0.0256'),
        DeclareLaunchArgument('sensor_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'resolution', default_value='0.005',
            description='Grid resolution, metres per cell'),
        DeclareLaunchArgument(
            'grid_margin', default_value='0.05',
            description='Metres of grid kept around the mat, for walls standing '
                        'at or just outside its edge'),
        DeclareLaunchArgument(
            'use_coverage_drive', default_value='true',
            description='Drive the pattern automatically. Turn it off to map by '
                        'teleop instead'),
        DeclareLaunchArgument(
            'row_spacing', default_value='0.05',
            description='Distance between passes, metres'),
    ]

    ld = LaunchDescription()
    for cmd in declare_cmds:
        ld.add_action(cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
