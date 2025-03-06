#ifndef ROBOT_AWAIT_HPP
#define ROBOT_AWAIT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_command/msg/command.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class RobotAwait : public rclcpp::Node
{
public:
    RobotAwait();

private:
    void command_callback(const custom_command::msg::Command::SharedPtr msg);
    void move_forward(float distance, float speed);
    void move_backward(float distance, float speed);
    void turn_left(float angle);
    void turn_right(float angle);
    void navigate_to(const std::string& position_name);
    void stop_robot();

    // Navigation2 Action 客户端
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
    void send_navigation_goal(const std::string& position_name);

    rclcpp::Subscription<custom_command::msg::Command>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

#endif // ROBOT_AWAIT_HPP