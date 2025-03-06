from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义 robot_await 节点
    robot_await_node = Node(
        package='robot_await',
        executable='robot_await',
        name='robot_await',
        output='screen',
        emulate_tty=True
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        robot_await_node
    ])