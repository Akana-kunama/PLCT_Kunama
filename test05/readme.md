# test 05

## description:

- 安装usb_cam驱动笔记本电脑自带的摄像头
- 新建功能包，编写节点实时订阅摄像头发布的图像话题消息并将ROS图像消息转换为OpenCV图像
- 在图像右上角绘制矩形，再将OpenCV图像转换回ROS图像消息重新发布到一个新的话题
- 用rviz或者rqt_image_view显示图像消息

## usb_cam install & test

- 安装usb_cam包： `sudo apt-get install ros-humble-usb-cam`
- 测试:
  -  `ros2 launch usb_cam camera.launch.py` 启动摄像头
  -  `ros2 run image_view image_view image:=/camera1/image_raw` 显示图像
  