# robot_navigation/launch/navigation_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    robot_navigation_pkg = get_package_share_directory('robot_navigation')

    # Path to the navigation parameters file
    nav2_params = os.path.join(robot_navigation_pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Launch the Nav2 bringup with the parameters
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[nav2_params]
        ),
    ])
