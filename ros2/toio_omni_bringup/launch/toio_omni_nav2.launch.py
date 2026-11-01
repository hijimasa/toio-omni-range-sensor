"""nav2 on the toio, on top of the cube and sensor bringup.

  ros2 launch toio_omni_bringup toio_omni_nav2.launch.py \
      cube_id:=a1b device_address:=AA:BB:CC:DD:EE:FF

Then give it a goal with the "2D Goal Pose" tool in RViz.

The command chain, which is the part worth having straight:

    controller_server / behavior_server --> cmd_vel_raw
                                              |
                                       collision_monitor
                                              |
                                            cmd_vel --> toio_ros2 --> cube

No velocity smoother: the cube driver already rate-limits and de-duplicates its
BLE writes, and the collision monitor's slowdown ramps rather than slams. Adding
one would be a third thing shaping the same command.

The frames are the other half. toio_ros2 publishes map -> center from the mat's
Position ID, which is absolute and drift-free, but nav2 insists on
map -> odom -> base. So the cube's transform is moved aside
(remap_cube_tf:=true, where it becomes /tf_gt) and toio_odom_bridge rebuilds the
chain from the same pose. `center` is used as the base frame throughout rather
than renaming anything to base_link.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('toio_omni_bringup')
    nav2_params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    default_map = os.path.join(bringup_dir, 'map', 'toio_a3_folded_left.yaml')
    default_params_file = os.path.join(
        bringup_dir, 'params', 'toio_a3_folded_left_params.yaml')

    namespace = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    cube_id = LaunchConfiguration('cube_id')
    device_address = LaunchConfiguration('device_address')
    ble_adapter = LaunchConfiguration('ble_adapter')
    sensor_x = LaunchConfiguration('sensor_x')
    sensor_y = LaunchConfiguration('sensor_y')
    sensor_z = LaunchConfiguration('sensor_z')
    sensor_yaw = LaunchConfiguration('sensor_yaw')
    use_rviz = LaunchConfiguration('use_rviz')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')

    declare_cmds = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the nodes and topics'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Occupancy map of the mat. Generate it with '
                        'tools/make_mat_map.py so it agrees with params_file'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Mat parameters for toio_ros2. Change this and the map '
                        'together -- a map that disagrees with the mat puts the '
                        'walls in the wrong place'),
        DeclareLaunchArgument('cube_id', default_value=''),
        DeclareLaunchArgument('device_address', default_value=''),
        DeclareLaunchArgument('ble_adapter', default_value=''),
        DeclareLaunchArgument('sensor_x', default_value='0.0'),
        DeclareLaunchArgument('sensor_y', default_value='0.0'),
        DeclareLaunchArgument('sensor_z', default_value='0.0256'),
        DeclareLaunchArgument('sensor_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_collision_monitor', default_value='true',
            description='Gate cmd_vel on the scan. Turning it off routes the '
                        'controller straight to cmd_vel, which is how you tell '
                        'a navigation problem apart from a sensor one -- the '
                        'monitor stops the robot outright when the scan goes '
                        'quiet, and that looks identical to nav2 not commanding'),
    ]

    bringup_args = {
        'namespace': namespace,
        'params_file': params_file,
        'cube_id': cube_id,
        'device_address': device_address,
        'ble_adapter': ble_adapter,
        'sensor_x': sensor_x,
        'sensor_y': sensor_y,
        'sensor_z': sensor_z,
        'sensor_yaw': sensor_yaw,
        'use_rviz': use_rviz,
        # Not optional here: the odom bridge owns odom -> center, so the cube's
        # own map -> center has to move out of the way or `center` gets two
        # parents.
        'remap_cube_tf': 'true',
        'use_collision_monitor': use_collision_monitor,
    }
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'toio_omni_bringup.launch.py')),
        launch_arguments=bringup_args.items())

    odom_bridge_node = Node(
        package='toio_omni_bringup',
        executable='toio_odom_bridge',
        name='toio_odom_bridge',
        namespace=namespace,
        output='screen')

    nav2_nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             namespace=namespace, output='screen',
             parameters=[nav2_params_file, {'yaml_filename': map_yaml}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', namespace=namespace, output='screen',
             parameters=[nav2_params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', namespace=namespace, output='screen',
             parameters=[nav2_params_file]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=namespace,
             output='screen', parameters=[nav2_params_file]),
    ]

    # The two nodes that emit velocity go either through the monitor or
    # straight at the cube. A remapping cannot be made conditional, so the
    # condition picks between whole actions.
    for pkg, exe in (('nav2_controller', 'controller_server'),
                     ('nav2_behaviors', 'behavior_server')):
        kwargs = dict(package=pkg, executable=exe, name=exe, namespace=namespace,
                      output='screen', parameters=[nav2_params_file])
        nav2_nodes.append(Node(condition=IfCondition(use_collision_monitor),
                               remappings=[('cmd_vel', 'cmd_vel_raw')], **kwargs))
        nav2_nodes.append(Node(condition=UnlessCondition(use_collision_monitor),
                               remappings=[('cmd_vel', 'cmd_vel_gated')], **kwargs))

    ld = LaunchDescription()
    for cmd in declare_cmds:
        ld.add_action(cmd)
    ld.add_action(bringup_launch)
    ld.add_action(odom_bridge_node)
    for node in nav2_nodes:
        ld.add_action(node)
    return ld
