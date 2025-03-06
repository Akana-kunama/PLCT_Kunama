# `ros_await`

description :  a package that  turns Encapsulated command to topic

```
.
├── CMakeLists.txt
├── include
│   └── robot_await
│       └── robot_await.hpp
├── launch
│   └── robot_await_launch.py
├── package.xml
├── readme.md
└── src
    └── robot_await_node.cpp
```

## `robot_await_launch.py`

启动节点 `robot_await_node` 



## `robot_await_node.cpp`

讲语义命令转换成 `ros2` 的相关指令，`message` 格式存放在 `custom_command`中

接口说明

+ `command_callback` 

  ```C++
  void RobotAwait::command_callback(const custom_command::msg::Command::SharedPtr msg);
  ```

  + `brief `  

    `RobotAwait` 类的成员函数，根据接收到的命令消息，解析命令并执行相应的操作。根据 `msg->name` 执行不同的机器人动作，例如前进、后退、转向或导航。若命令无效或参数不足，记录错误并停止机器人。

  + `param` : 
    + `msg`: 包含命令名称和参数的消息。

+ `move_forward`  / `move_backward`

  ```C++
  void RobotAwait::move_forward(float distance, float speed);
  void RobotAwait::move_backward(float distance, float speed);
  ```

  + `brief` 

    控制机器人以指定速度前进/后退指定的距离。

  + `param` 

    + `distance` 机器人需要前进的距离（米）。
    + `speed` 机器人前进的速度（米/秒）。

+ `turn_right` / `turn_left`

  ```C++
  void RobotAwait::turn_left(float angle);
  void RobotAwait::turn_right(float angle);
  ```

  + `brief` 

    控制机器人左转/右转指定的角度。

  + `param` 

    + `angle`: 机器人右转的角度（度）。

+ `navigate_to`

  ```c++
  void RobotAwait::navigate_to(const std::string& position_name);
  ```

  + `brief` 

    控制机器人到行至特定的位置，所有位置预先留存在 `navigation_goals.json`中。

  + `param` 

    + `position_name`: 目标位置的名称



## 节点测试

启动节点 ：

```bash
ros2 launch robot_await robot_await_launch.py
```

前行测试 ：

```bash
ros2 topic pub /command_topic custom_command/msg/Command "{name: 'MoveForward', parameters: ['2.0', '0.5']}"
```

转弯测试 ：+

```bash
ros2 topic pub /command_topic custom_command/msg/Command "{name: 'TurnLeft', parameters: ['90.0']}"
```

定点导航测试：

```bash
ros2 topic pub /command_topic custom_command/msg/Command "{name: 'NavigateTo', parameters: ['Home']}"
```

