import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

 
def generate_launch_description():
    # Declare the launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file = LaunchConfiguration("map_file")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation/Gazebo clock"
    )

    # Declare the 'map_file' argument
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("robot_slam"),
            "maps",
            "saved_map.yaml"
        ]),
        description="Full path to the yaml map file"
    )

    # Declare the 'amcl_params_file' argument
    amcl_params_file_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("robot_slam"),
            "config",
            "amcl.config.yaml"
        ]),
        description="Full path to the ROS2 parameters file to use for the amcl node",
    )

    # Create the Map Server node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
    )

    # Create the AMCL node
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[amcl_params_file, {"use_sim_time": use_sim_time}],
    )

    # Create the Static Transform Publisher node for map -> odom
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )


    # Create the Lifecycle Manager node
    nav_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav_manager",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    # Log the AMCL parameters file being used
    log_amcl_params_cmd = LogInfo(
        msg=['Using AMCL parameters file: ', amcl_params_file]
    )

    return LaunchDescription([
        use_sim_time_arg,
        amcl_params_file_arg,
        map_file_arg,
        map_server_node,
        amcl_node,
        static_tf_map_to_odom ,
        nav_manager,
        log_amcl_params_cmd
    ])


if __name__ == '__main__':
    generate_launch_description()