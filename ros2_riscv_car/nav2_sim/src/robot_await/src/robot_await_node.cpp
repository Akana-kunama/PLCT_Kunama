#include "robot_await/robot_await.hpp"
#include <chrono>
#include <thread>
#include <memory>

using namespace std::chrono_literals;

RobotAwait::RobotAwait() : Node("robot_await")
{
    // 创建订阅者
    subscription_ = this->create_subscription<custom_command::msg::Command>(
        "command_topic", 10,
        std::bind(&RobotAwait::command_callback, this, std::placeholders::_1));

    // 创建发布者
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 创建 Navigation2 Action 客户端
    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

void RobotAwait::command_callback(const custom_command::msg::Command::SharedPtr msg)
{
    std::string name = msg->name;
    std::vector<std::string> parameters = msg->parameters;

    if (name == "MoveForward")
    {
        if (parameters.size() >= 2)
        {
            float distance = std::stof(parameters[0]);
            float speed = std::stof(parameters[1]);
            RCLCPP_INFO(this->get_logger(), "Executing MoveForward: distance=%.2f m, speed=%.2f m/s", distance, speed);
            move_forward(distance, speed);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "MoveForward requires 2 parameters: distance and speed");
        }
    }
    else if (name == "MoveBackward")
    {
        if (parameters.size() >= 2)
        {
            float distance = std::stof(parameters[0]);
            float speed = std::stof(parameters[1]);
            RCLCPP_INFO(this->get_logger(), "Executing MoveBackward: distance=%.2f m, speed=%.2f m/s", distance, speed);
            move_backward(distance, speed);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "MoveBackward requires 2 parameters: distance and speed");
        }
    }
    else if (name == "TurnLeft")
    {
        if (parameters.size() >= 1)
        {
            float angle = std::stof(parameters[0]);
            RCLCPP_INFO(this->get_logger(), "Executing TurnLeft: angle=%.2f degrees", angle);
            turn_left(angle);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "TurnLeft requires 1 parameter: angle");
        }
    }
    else if (name == "TurnRight")
    {
        if (parameters.size() >= 1)
        {
            float angle = std::stof(parameters[0]);
            RCLCPP_INFO(this->get_logger(), "Executing TurnRight: angle=%.2f degrees", angle);
            turn_right(angle);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "TurnRight requires 1 parameter: angle");
        }
    }
    else if (name == "NavigateTo")
    {
        if (parameters.size() >= 1)
        {
            std::string position_name = parameters[0];
            RCLCPP_INFO(this->get_logger(), "Executing NavigateTo: positionName=%s", position_name.c_str());
            navigate_to(position_name);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "NavigateTo requires 1 parameter: positionName");
        }
    }
    else
    {
        // 默认指令：停止机器人
        RCLCPP_WARN(this->get_logger(), "Unknown command: %s. Stopping robot.", name.c_str());
        stop_robot();
    }
}

void RobotAwait::move_forward(float distance, float speed)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = speed;  // 设置线速度
    msg.angular.z = 0.0;   // 设置角速度为 0

    // 计算运行时间
    float time = distance / speed;
    auto start_time = this->now();

    // 发布速度命令
    while ((this->now() - start_time).seconds() < time)
    {
        cmd_vel_publisher_->publish(msg);
        std::this_thread::sleep_for(100ms);
    }

    // 停止机器人
    stop_robot();
}

void RobotAwait::move_backward(float distance, float speed)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = -speed;  // 设置线速度为负值
    msg.angular.z = 0.0;    // 设置角速度为 0

    // 计算运行时间
    float time = distance / speed;
    auto start_time = this->now();

    // 发布速度命令
    while ((this->now() - start_time).seconds() < time)
    {
        cmd_vel_publisher_->publish(msg);
        std::this_thread::sleep_for(100ms);
    }

    // 停止机器人
    stop_robot();
}

void RobotAwait::turn_left(float angle)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;      // 设置线速度为 0
    msg.angular.z = 0.5;     // 设置角速度（左转）

    // 计算运行时间（假设角速度为 0.5 rad/s）
    float time = angle * (M_PI / 180.0) / 0.5;  // 将角度转换为弧度
    auto start_time = this->now();

    // 发布速度命令
    while ((this->now() - start_time).seconds() < time)
    {
        cmd_vel_publisher_->publish(msg);
        std::this_thread::sleep_for(100ms);
    }

    // 停止机器人
    stop_robot();
}

void RobotAwait::turn_right(float angle)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;      // 设置线速度为 0
    msg.angular.z = -0.5;    // 设置角速度（右转）

    // 计算运行时间（假设角速度为 0.5 rad/s）
    float time = angle * (M_PI / 180.0) / 0.5;  // 将角度转换为弧度
    auto start_time = this->now();

    // 发布速度命令
    while ((this->now() - start_time).seconds() < time)
    {
        cmd_vel_publisher_->publish(msg);
        std::this_thread::sleep_for(100ms);
    }

    // 停止机器人
    stop_robot();
}

void RobotAwait::navigate_to(const std::string& position_name)
{
    // 调用 Navigation2 的 NavigateToPose Action
    send_navigation_goal(position_name);
}

void RobotAwait::send_navigation_goal(const std::string& position_name)
{
    // 检查 Action 服务器是否可用
    if (!navigation_client_->wait_for_action_server(5s))
    {
        RCLCPP_ERROR(this->get_logger(), "Navigation action server not available after waiting");
        return;
    }

    // 创建目标位置
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";  // 假设目标位置在 map 坐标系中

    // 根据 position_name 设置目标位置
    if (position_name == "Home")
    {
        goal_msg.pose.pose.position.x = 0.0;
        goal_msg.pose.pose.position.y = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;
    }
    else if (position_name == "Office")
    {
        goal_msg.pose.pose.position.x = 2.0;
        goal_msg.pose.pose.position.y = 3.0;
        goal_msg.pose.pose.orientation.w = 1.0;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown position name: %s", position_name.c_str());
        return;
    }

    // 发送目标
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](const GoalHandleNavigateToPose::SharedPtr& goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    };

    send_goal_options.result_callback =
        [this](const GoalHandleNavigateToPose::WrappedResult& result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Navigation was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    };

    navigation_client_->async_send_goal(goal_msg, send_goal_options);
}

void RobotAwait::stop_robot()
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;  // 设置线速度为 0
    msg.angular.z = 0.0; // 设置角速度为 0
    cmd_vel_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Robot stopped.");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotAwait>());
    rclcpp::shutdown();
    return 0;
}