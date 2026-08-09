"""Bring up the range sensor on its own.

This is the standalone view: no cube, no map, just the sensor and RViz, which is
what you want when checking that the unit is seen and the scan looks sane.

  ros2 launch toio_omni_range omni_range.launch.py
  ros2 launch toio_omni_range omni_range.launch.py device_address:=AA:BB:CC:DD:EE:FF

When the cube driver is running too, set use_dummy_tf:=false and let the robot
description place omni_link -- the identity transform published here exists only
so RViz has a frame to draw in.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('toio_omni_range')
    calib_params = os.path.join(pkg_dir, 'params', 'omni_range_calib.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'omni_range.rviz')

    namespace = LaunchConfiguration('namespace')
    device_address = LaunchConfiguration('device_address')
    ble_adapter = LaunchConfiguration('ble_adapter')
    frame_id = LaunchConfiguration('frame_id')
    use_rviz = LaunchConfiguration('use_rviz')
    use_dummy_tf = LaunchConfiguration('use_dummy_tf')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace of the sensor node and its topics (e.g. toio1)')
    declare_device_address_cmd = DeclareLaunchArgument(
        'device_address', default_value='',
        description='BLE address of the unit. Set it to skip the discovery '
                    'scan, which is what disturbs the cube on the same adapter')
    declare_ble_adapter_cmd = DeclareLaunchArgument(
        'ble_adapter', default_value='',
        description='BlueZ adapter, e.g. hci1. Empty uses the default')
    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='omni_link',
        description='Frame the LaserScan is expressed in')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Use RViz2 if true')
    declare_use_dummy_tf_cmd = DeclareLaunchArgument(
        'use_dummy_tf', default_value='true',
        description='Publish an identity map -> frame_id transform so the scan '
                    'is displayable without a robot. Turn it off once the cube '
                    'driver provides the real one')

    omni_range_node = Node(
        package='toio_omni_range',
        executable='omni_range_node',
        name='omni_range_node',
        namespace=namespace,
        parameters=[
            calib_params,
            {'device_address': device_address,
             'ble_adapter': ble_adapter,
             'frame_id': frame_id}],
        output='screen')

    dummy_tf_node = Node(
        condition=IfCondition(use_dummy_tf),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='omni_dummy_tf',
        arguments=['--frame-id', 'map', '--child-frame-id', frame_id],
        output='screen')

    rviz2_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_device_address_cmd)
    ld.add_action(declare_ble_adapter_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_dummy_tf_cmd)

    ld.add_action(omni_range_node)
    ld.add_action(dummy_tf_node)
    ld.add_action(rviz2_node)
    return ld
