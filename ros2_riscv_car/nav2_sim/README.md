# Yahboom RISC-V ROS Robotcar Mapping & Navigation Simulation

Description : A simulation packgage based on Gazebo and Navigation2 for Yahboom RISC-V ROS Robotcar

System : Ubuntu 22.04


## robot discription

Desicription : to display robot and the simulation environment

`ros2 launch robot_description robot_display_launch.py`

+ display robot and environment
+ able to drive the robot with teleop_twist_keyboard
+ able to scan the environment with radar


![](pics/robot_display3.png.png)

![](pics/robot_displayr4.png)


***


`ros2 launch robot_slam slam_mapping_launch.py`

 + able to mapping with slam_toolbox

`ros2 launch robot_slam map_saving_launch.py`
 + able to save map to specified path

`ros2 launch robot_slam acml_launch.py`
 + able to load saved map 

![](pics/robot_mapping_load_and_save.png)
