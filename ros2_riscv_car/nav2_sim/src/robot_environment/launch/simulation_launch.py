import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_environment_pkg = get_package_share_directory('robot_environment')
    robot_control_pkg = get_package_share_directory('robot_control')
    # Path to the world file
    world_file = os.path.join(robot_environment_pkg, 'worlds', 'sim_world.world')

    # Path to the robot's URDF
    urdf_file = os.path.join(robot_description_pkg, 'urdf', 'MicroROS.urdf')


    # Path to the robot control config
    controller_config_file = os.path.join(robot_control_pkg, 'config', 'robot_controllers.yaml')

    # Read the URDF file contents into a string
    with open(urdf_file, 'r') as urdf_file_obj:
        robot_description_content = urdf_file_obj.read()

    # Set the custom GAZEBO_MODEL_PATH
    gazebo_model_path = os.path.join(os.getcwd(), 'src', 'robot_description', 'meshes')

    return LaunchDescription([
        # Set GAZEBO_MODEL_PATH environment variable, including default model path
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', '/home/kunama/PLCT/NAV2/nav2_sim/src/robot_description/meshes'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', 'file:///'),

        # Set GAZEBO_PLUGIN_PATH to ensure Gazebo can find ROS plugins
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib'),

        # Log the URDF file path for debugging
        LogInfo(msg="Using URDF file at: " + urdf_file),

        # Launch Gazebo server with GazeboRosFactory plugin
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzserver',
            arguments=[world_file, '--verbose',
                    #    '-s', 'libgazebo_ros_init.so',
                       '-s', 'libgazebo_ros_factory.so',
                       '--plugin', 'libgazebo_ros2_control.so'],
            output='screen'
        ),
        # Launch Gazebo client
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzclient',
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'micro4.0', '-x', '1.0', '-y', '-1.0', '-z', '0.0', '-Y', '0.0'],  # Set initial position and orientation
            # arguments=['-file', urdf_file, '-entity', 'micro4.0'],
            output='screen'
        ),


    ])
