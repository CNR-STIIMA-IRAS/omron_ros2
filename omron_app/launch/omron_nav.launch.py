import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node,SetRemap
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    declared_arguments = []


    log_level = LaunchConfiguration('log_level')

    declared_arguments.append(
        DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'))

    declared_arguments.append(
            DeclareLaunchArgument(
        'nav',
        default_value='True',
        description='Whether run nav too'))

    nav                      = LaunchConfiguration('nav')

    omron_driver = Node(
            package='omron_ros2_agv',
            executable='omron_ros2_agv_node',
            namespace='omron',
            remappings= [('/omron/map', '/map'),('/omron/map_metadata','/map_metadata')],
            output='screen')

    pcl_to_ls =    Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings={('cloud_in', 'omron/cloud_in'),('scan', 'omron/scan')},
            parameters=[{
                'target_frame': 'omron/base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -2.35,  # -M_PI/2
                'angle_max':  2.35,  # M_PI/2
                'angle_increment': 0.007,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 24.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )

    laser_throttle = Node(
            package='topic_tools',
            executable='throttle',
            namespace='omron',
            parameters=[{
                'input_topic': 'scan',
                'throttle_type': 'messages',
                'msgs_per_sec': 2.0,
                'output_topic': 'scan_rviz'
            }],
            arguments=['messages scan 2 scan_rviz'],
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
        condition=IfCondition(nav),
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

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("omron_app"), "urdf", "system.urdf.xacro"]),
            " ","name:=","omron",
            " ","ur_type:=","ur10",
            " ","prefix:=","omron/",
        ]
    )

    robot_description = {"robot_description": robot_description_content}


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace='omron',
        output="screen",
        # parameters=[robot_description,{"frame_prefix": 'omron'}],
        parameters=[robot_description],
    )

    nodes_to_start = [
                omron_driver,
                pcl_to_ls,
                laser_throttle,
                robot_state_publisher_node,
                TimerAction(
                period=1.0,
                actions=[load_nodes],
                ),
                ]

    return LaunchDescription(declared_arguments + nodes_to_start)