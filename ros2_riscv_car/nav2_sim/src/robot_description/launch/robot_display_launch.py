import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import launch_ros
from launch_ros.actions import Node

def generate_launch_description():
    # -------------------------
    # 1. Launch Configuration
    # -------------------------
    # Declare launch arguments

    # Declare the 'use_sim_time' argument
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model')
    rviz_config_path = LaunchConfiguration('rvizconfig')



    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )
    


    model_path_declare= DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            get_package_share_directory('robot_description'),
            'urdf',
            'MicroROS.urdf'
        ),
        description='Absolute path to robot URDF file'
    )

    rviz_config_path_declare = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(
            get_package_share_directory('robot_description'),
            'rviz',
            'urdf_config.rviz'
        ),
        description='Absolute path to RViz config file'
    )

    # -------------------------
    # 2. Paths and File Configurations
    # -------------------------
    # Get package share directories

    robot_description_pkg = get_package_share_directory('robot_description')
    robot_environment_pkg = get_package_share_directory('robot_environment')

    # Define file paths
    # world_file = os.path.join(robot_environment_pkg, 'worlds', 'sim_world.world')
    world_file = os.path.join(robot_environment_pkg, 'worlds', 'ROS-Academy.world')
    urdf_file_path = os.path.join(robot_description_pkg, 'urdf', 'MicroROS.urdf')
    default_rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'urdf_config.rviz')

    # --------------------------
    # Read the URDF file content
    # --------------------------
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()



    # -------------------------
    # 3. Nodes Definitions
    # -------------------------
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': urdf_content},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
        # Uncomment below to enable GUI
        # condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    # Spawn Entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'micro4.0',
            '-x', '1.0',
            '-y', '-1.0',
            '-z', '0.0',
            '-Y', '0.0',
            '-topic', 'robot_description'
        ]
    )


     # -------------------------
    # 4. Launch Gazebo
    # -------------------------
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    # -------------------------
    # 5. Assemble Launch Description
    # -------------------------
    return LaunchDescription([
        # Declare Launch Arguments
        use_sim_time_declare,
        model_path_declare,
        rviz_config_path_declare,

        # Launch Gazebo
        gazebo_launch,

        # Nodes
        joint_state_publisher_node,
        robot_state_publisher_node,
        # robot_localization_node,
        spawn_entity,
        rviz_node
    ])