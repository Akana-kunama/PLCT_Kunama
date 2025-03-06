import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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
    # Declare the launch arguments
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_saving_path = LaunchConfiguration('map_saving_path')
    world_file = LaunchConfiguration('world')

    
    # Declare the 'slam_params_file' argument
    slam_params_file_declare = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('robot_slam'), 'config', 'slam_mapping_params.yaml']
        ),
        description='Full path to the SLAM parameters file to use'
    )

    # Define the default map saving path
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("robot_slam"), 'maps', 'saved_map.yaml']
    )

    map_saving_path_declare = DeclareLaunchArgument(
        'map_saving_path',
        default_value= default_map_path ,
        description='the path to save the map'
    )


    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='use simulation/Gazebo clock'
    )

    world_file_declare = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('robot_environment'),
            'worlds',
            'sim_world.world'
        ),
        description='Absolute path to the world file to load'
    )

    # display robot and enviornment with robot display
     # Include the robot_display_launch
    robot_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'robot_display_launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file
        }.items()
    )


    # OpaqueFunction to ensure the map saving directory exists
    ensure_map_dir_action = OpaqueFunction(function=ensure_map_directory)

    # Create the SLAM Toolbox node


    slam_toolbox_node = Node(    # node for not using saved map
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file,{'map_file_name':''}]
    )

    # Log the SLAM parameters file being used
    log_slam_params_cmd = LogInfo(
        msg=['Using SLAM parameters file: ', slam_params_file]
    )

    return LaunchDescription([
        slam_params_file_declare,
        map_saving_path_declare,
        use_sim_time_declare,
        world_file_declare,

        robot_display_launch,
        
        ensure_map_dir_action,
        slam_toolbox_node,
        log_slam_params_cmd
    ])

if __name__ == '__main__':
    generate_launch_description()