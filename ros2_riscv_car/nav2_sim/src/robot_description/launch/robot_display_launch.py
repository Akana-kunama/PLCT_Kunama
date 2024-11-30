import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    # Get the path to the package's share directory using get_package_share_directory
    package_share_directory = get_package_share_directory('robot_description')
    robot_environment_pkg = get_package_share_directory('robot_environment')
    world_file = os.path.join(robot_environment_pkg, 'worlds', 'sim_world.world')


    # Define the path to the URDF file
    urdf_file_path = os.path.join(package_share_directory, 'urdf', 'MicroROS.urdf')
    default_rviz_config_path = os.path.join(package_share_directory, 'rviz/urdf_config.rviz')

    # Read the URDF file content
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 
                   'micro4.0', '-x', '1.0', '-y', '-1.0', '-z', '0.0', '-Y', '0.0',
                   '-topic', 
                   'robot_description'],
        output='screen'
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(package_share_directory, 'config/ekf.yaml')]
)
    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
        #                                     description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_file_path ,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', 
                                           '-s', 'libgazebo_ros_init.so', 
                                           '-s', 'libgazebo_ros_factory.so',
                                           world_file], 
                                        output='screen'),
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        robot_localization_node,
        spawn_entity,
        rviz_node
    ])