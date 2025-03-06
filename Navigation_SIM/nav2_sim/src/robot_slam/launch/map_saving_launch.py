import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def ensure_map_directory(context, *args, **kwargs):
    map_saving_path = LaunchConfiguration('map_saving_path').perform(context)
    # Extract the directory part from the path
    map_dir = os.path.dirname(map_saving_path)
    if not os.path.exists(map_dir):
        os.makedirs(map_dir)
        print(f"[INFO] Created map saving directory: {map_dir}")
    else:
        print(f"[INFO] Map saving directory already exists: {map_dir}")
    return []  # OpaqueFunction requires returning a list of actions, but here we do nothing



def generate_launch_description():
    
    map_saving_path = LaunchConfiguration('map_saving_path')

    # Define the default map saving path
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("robot_slam"), 'maps', 'saved_map']
    )

    map_arg = DeclareLaunchArgument(
                                    name='map_path',
                                    default_value=default_map_path,
                                    description='The path of the map'
                                    )

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        arguments=[
            '-f', LaunchConfiguration('map_path'), 
            '--ros-args', '-p', 'save_map_timeout:=60000.00'],
    )


    return LaunchDescription([
        map_arg,
        map_saver_node
    ])