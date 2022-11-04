
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    declared_arguments = []


    log_level = LaunchConfiguration('log_level')

    declared_arguments.append(
        DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'))

    omron_driver = Node(
            package='omron_ros2_agv',
            executable='omron_ros2_agv_node',
            name='omron_ros2_agv_node',
            namespace='omron',
            output='screen')

    nav_sw1_params = os.path.join(
            get_package_share_directory('omron_app'),
            'config',
            "nav_params.yaml")

    configured_params = RewrittenYaml(
        source_file=nav_sw1_params,
        root_key="omron",
        param_rewrites={'autostart': "True"},
        convert_types=True)

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']
                    #    'velocity_smoother']

    load_nodes = GroupAction(
        actions=[PushRosNamespace('omron'),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings= [('cmd_vel', '/omron/cmd_vel')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            # Node(
            #     package='nav2_velocity_smoother',
            #     executable='velocity_smoother',
            #     name='velocity_smoother',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings +
            #             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

#     nav_sw1 = GroupAction(
#         actions=[IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),"launch", 'navigation_launch.py')),
#                 launch_arguments={'namespace': "omron",
#                                 'use_sim_time': "true",
#                                 'autostart': "true",
#                                 'params_file': nav_sw1_params,
#                                 'use_lifecycle_mgr': 'false',
#                                 'log_level': 'info',
#                                 'map_subscribe_transient_local': 'true'}.items())])

    nodes_to_start = [
                omron_driver,
                TimerAction(
                period=2.0,
                actions=[load_nodes],
                ),
                ]

    return LaunchDescription(declared_arguments + nodes_to_start)