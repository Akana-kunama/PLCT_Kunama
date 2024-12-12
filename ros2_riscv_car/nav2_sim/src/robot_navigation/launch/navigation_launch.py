# navigation_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    # Declare launch arguments
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
            "config",
            "nav2_params.yaml"
        ]),
        description="Full path to the Nav2 parameters file to use for the navigation stack"
    )

    # Map Server node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
    )

    # AMCL node
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params_file],
    )

    # Planner Server node
    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_file],
    )

    # Controller Server node
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_file],
    )

    # Behavior Tree Navigator node
    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_file],
    )

    # Recoveries Server node
    recoveries_server_node = Node(
        package="nav2_recoveries",
        executable="recoveries_server",
        name="recoveries_server",
        output="screen",
        parameters=[nav2_params_file],
    )

    # Lifecycle Manager node
    lifecycle_manager_node = Node(
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
                "bt_navigator",
                "recoveries_server"
            ],
        }],
    )

    # RViz node (Optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("robot_slam"),
        "config",
        "nav2_default_view.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Log Info
    log_cmd = LogInfo(
        msg=["Launching Navigation Stack with map: ", map_file]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_file_cmd,
        declare_nav2_params_file_cmd,
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        bt_navigator_node,
        recoveries_server_node,
        lifecycle_manager_node,
        # Uncomment the following line to launch RViz automatically
        # rviz_node,
        log_cmd
    ])
