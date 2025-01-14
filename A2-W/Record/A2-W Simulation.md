# A2-W Simulation Record

## publish robot model to rviz

1. build the package with colcon in folder `resource` :

   ```bash
   cd <path to a2_w_t3_mujoco_simulator/resource>
   colcon build --symlink-install
   ```

2. launch ros2 package `robot_model` to publish robot model

   ```bash
   ros2 launch robot_model raise_a2_w_t3.launch.py debug_urdf:=true
   ```

   ![rviz_publish](../../../../PLCT/A2-W/Record/pics/rviz_publish.png)

3. able to modify robot state with Joint State publisher gui

   ![rviz_random_state](../../../../PLCT/A2-W/Record/pics/rviz_random_state.png)







## publish robot model to mujoco

1. install gcc-12 and g++-13

   ```bash
   sudo apt-get update
   sudo apt-get install gcc-13 g++-13
   ```

2. launch bash script under folder `bin`

   ```bash
   cd <path to a2_w_t3_mujoco_simulator/bin>
   ./start_sim.sh -s
   ```

   ![mujoco_start_bash](../../../../PLCT/A2-W/Record/pics/mujoco_start_bash.png)

   choose `raise_a2_w_t3 `

   ![mujoco_publish](../../../../PLCT/A2-W/Record/pics/mujoco_publish.png)



## file structure

```
./a2_w_t3_mujoco_simulator
├── bin
├── configuration
├── git
├── lib
├── local
├── resource
└── share
```

+ bin

  ```
  ./a2_w_t3_mujoco_simulator/bin/
  ├── aima-sim-app-main
  ├── aimrte
  ├── aimrt_main
  ├── cfg
  ├── libaimrt_log_control_plugin.so
  ├── libaimrt_mqtt_plugin.so
  ├── libaimrt_net_plugin.so
  ├── libaimrt_opentelemetry_plugin.so
  ├── libaimrt_parameter_plugin.so
  ├── libaimrt_ros2_plugin.so
  ├── libaimrt_time_manipulator_plugin.so
  ├── log
  ├── MJDATA.TXT
  ├── MUJOCO_LOG.TXT
  ├── protoc
  ├── protoc-3.21.12.0
  ├── protoc_plugin_cpp_gen_aimrt_cpp_rpc
  ├── protoc_plugin_py_gen_aimrt_cpp_rpc.py
  ├── ros2_py_gen_aimrt_cpp_rpc.py
  ├── sim_configuration_directory_cache.txt
  ├── sim_robot_name_cache.txt
  └── start_sim.sh
  
  ```

  + `start_sim.sh`

    `mujoco`仿真环境启动脚本

    Key Path:

    + 机器人模型文件位置：`a2_w_t3_mujoco_simulator/resource/model`

    + 仿真执行文件：`a2_w_t3_mujoco_simulator/aima-sim-app-main`

    + 仿真配置文件：`a2_w_t3_mujoco_simulator/configuration/robot/raise_a2_w_t3/simulator/default.yaml`

+ configuration

  ```
  configuration/
  └── robot
      └── raise_a2_w_t3
          ├── init_parameters
          │   └── default.yaml
          ├── 
          │   └── default.yaml
          └── simulator
              ├── default.yaml
              └── default.yaml.dump
  
  ```

  + `configuration/robot/raise_a2_w_t3/simulator/default.yaml` `mujoco`仿真配置文件

+ resource

  ```
  resource/
  ├── build
  │   ├── COLCON_IGNORE
  │   └── robot_model
  ├── CMakeLists.txt
  ├── config
  │   └── zeros.yaml
  ├── install
  │   ├── COLCON_IGNORE
  │   ├── local_setup.bash
  │   ├── local_setup.ps1
  │   ├── local_setup.sh
  │   ├── _local_setup_util_ps1.py
  │   ├── _local_setup_util_sh.py
  │   ├── local_setup.zsh
  │   ├── robot_model
  │   ├── setup.bash
  │   ├── setup.ps1
  │   ├── setup.sh
  │   └── setup.zsh
  ├── launch
  │   └── raise_a2_w_t3.launch.py
  ├── log
  │   ├── build_2025-01-14_10-17-05
  │   ├── COLCON_IGNORE
  │   ├── latest -> latest_build
  │   └── latest_build -> build_2025-01-14_10-17-05
  ├── meshes
  │   └── raise_a2_w_t3
  ├── model
  │   ├── environment
  │   ├── raise_a2_w_t3_flat.xml
  │   └── robot
  ├── package.xml
  ├── rviz
  │   └── raise_a2_w_t3.rviz
  ├── terrain
  │   ├── flat_ground.png
  │   ├── hurdles.png
  │   ├── map_invert.png
  │   ├── slope_20deg.png
  │   ├── stair_16x28.png
  │   ├── stepping_stones.png
  │   ├── step.png
  │   └── wave_heightmap.png
  └── urdf
      └── raise_a2_w_t3
  
  ```

  + `launch` 启动文件文件夹
    + `launch/raise_a2_w_t3.launch.py`: 机器人模型可视化启动文件
  + `resource/model`机器人仿真模型文件
    + `./robot`: 机器人模型
    + `./environment` 仿真环境

  + `./rviz` `rviz2` 启动配置
