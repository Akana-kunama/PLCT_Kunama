# navigation_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, OpaqueFunction,GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # -------------------------
    # 1. Declare Launch Arguments
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]




    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",  # Set to "true" for simulation
        description="Use simulation/Gazebo clock"
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("robot_slam"),
            "maps",
            "saved_map.yaml"
        ]),
        description="Full path to the map YAML file to use for navigation"
    )

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("robot_navigation"),
            "params",
            "nav2_params.yaml"
        ]),
        description="Full path to the Nav2 parameters file to use for the navigation stack"
    )




    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    
    # -------------------------
    # 2. Include Robot Display Launch
    # -------------------------

    robot_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'robot_display_launch.py'
            ])
        ]),
        launch_arguments={
            # Add other launch arguments if necessary
        }.items()
    )


    # -------------------------
    # 4. Define Nav2 Nodes Group
    # -------------------------

    nav2_nodes = GroupAction(
         actions=[
             
            # Map Server node
             Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
            ),

             # AMCL node
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[nav2_params_file],
            ),

            # Planner Server node
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),

            # Controller Server node
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # Behavior Server node
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # Behavior Tree Navigator node
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            ),


            # -----------------------
            # Lifecycle Manager node
            # -----------------------
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": [
                        "map_server",
                        "amcl",
                        "planner_server",
                        "controller_server",
                        "behavior_server",
                        "bt_navigator",
                    ],
                }],
            ),

            # Log Info
            LogInfo(msg=["Launching Navigation Stack with map: ", map_file]),
            LogInfo(msg=["AMCL node started"])

         ]
    )
    

    


    


    return LaunchDescription([
        # robot_description_launch,
        robot_display_launch,
        declare_use_sim_time_cmd,
        declare_map_file_cmd,
        declare_nav2_params_file_cmd,
        declare_log_level_cmd,
        nav2_nodes
    ])
