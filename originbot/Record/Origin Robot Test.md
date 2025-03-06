# Origin Robot Test

## **部件组装**

组装流程：http://originbot.org/guide/hardware_setup/

## **镜像与固件烧录**

**SD卡镜像烧录**

- 安装 `balenaEtcher`

  ```
  weget -c https://github.com/balena-io/etcher/releases/download/v1.19.25/balena-etcher_1.19.25_amd64.deb
  sudo apt install ./balena-etcher_1.19.25_amd64.deb
  ```

+ 下载并解压对应镜像：
  ![image-20250116162440931](../../../.var/app/io.typora.Typora/config/Typora/typora-user-images/image-20250116162440931.png)

  ![wechat_2025-01-23_140252_339](./pics/wechat_2025-01-23_140252_339.png)

  用 `balenaEther` 将对应镜像烧录入SD卡中即可
  
  ![image-20250116095522359](./pics/image-20250116095522359.png)
  

**控制板固件烧录**

+ 下载 `flymcu.exe`

  https://gitee.com/guyuehome/originbot_controller/blob/master/tools/FlyMcu.exe

+ 下载对应控制固件：

  https://pan.baidu.com/s/1qMlekq84JQBM8OZC3pm0rA?pwd=gyh1

+ 烧录配置：

  http://originbot.org/guide/firmware_install/#download_controller_firmware

## 基本功能测试

**底盘启动测试**

```
ros2 launch originbot_bringup originbot.launch.py
```

**相机驱动测试**

```bash
cd /userdata/dev_ws
ros2 launch originbot_bringup mipi_camera_websoket_display.launch.py
```

![Screenshot from 2025-01-16 11-44-40](./pics/Screenshot from 2025-01-16 11-44-40.png)

![camera](./pics/camera.png)

**雷达驱动**

+ 机器人端：

  ```bash
  ros2 launch originbot_bringup originbot.launch.py use_lidar:=true
  ```

  ![wechat_2025-01-16_171249_551](./pics/wechat_2025-01-16_171249_551.png)



+ 上位机：

  ```
  ros2 launch originbot_viz display_lidar.launch.py
  ```

  ![wechat_2025-01-16_171255_118](./pics/wechat_2025-01-16_171255_118.png)

**`IMU`驱动**

+ 机器人端：

  ```
  ros2 launch originbot_bringup originbot.launch.py use_imu:=true
  ```

  ![wechat_2025-01-16_171410_036](./pics/wechat_2025-01-16_171410_036.png)



+ 上位机：

  ```
  ros2 launch originbot_viz display_imu.launch.py
  ```

  ![wechat_2025-01-16_171415_190](./pics/wechat_2025-01-16_171415_190.png)

**地图构建**

+ 机器人端：

  ```
  ros2 launch originbot_bringup originbot.launch.py use_lidar:=true use_imu:=true
  ```

  ![navigation_bringup_sucess](./pics/navigation_bringup_sucess.png)

  ```
  ros2 launch originbot_navigation cartographer.launch.py
  ```

  ![navigation_carto_sucess](./pics/navigation_carto_sucess.png)

+ 上位机：

  ```
  ros2 launch originbot_viz display_slam.launch.py
  ```

  ![navigation_rviz_sucess](./pics/navigation_rviz_sucess.png)

  ```
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

  可通过键盘控制轮子转动

+ 机器人端：

  ```
  cd /userdata/dev_ws/src/originbot/originbot_navigation/maps
  ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p save_map_timeout:=10000.0
  ```

![mapping_frames](./pics/mapping_frames.png)

**自主导航**

+ 机器人端：

  ```
  $ cd /userdata/dev_ws/
  $ colcon build --symlink-install --packages-select originbot_navigation
  ```

  ```
  ros2 launch originbot_bringup originbot.launch.py use_lidar:=true use_imu:=true
  ```

  ```
  ros2 launch originbot_navigation nav_bringup.launch.py
  ```

  ![wechat_2025-01-23_140133_836](./pics/wechat_2025-01-23_140133_836.png)

   map frame 缺失

+ 上位机：

  ```
  ros2 launch originbot_viz display_navigation.launch.py
  ```

  





