import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the package's share directory using get_package_share_directory
    package_share_directory = get_package_share_directory('robot_description')
    
    # Define the path to the URDF file
    urdf_file_path = os.path.join(package_share_directory, 'urdf', 'MicroROS.urdf')
    
    # Read the URDF file content
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    # The launch description
    return LaunchDescription([
        # Log the path to the URDF file for debugging
        LogInfo(
            msg="URDF path: " + urdf_file_path
        ),
        
        # Launch robot_state_publisher node to read the URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        ),
    ])
