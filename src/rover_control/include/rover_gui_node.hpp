#ifndef ROVER_GUI_NODE_HPP
#define ROVER_GUI_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

class RoverDashboard;

// ROS Node for GUI
class RoverGUINode : public rclcpp::Node {
public:
    explicit RoverGUINode(RoverDashboard *dashboard);

private:
    void driveCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void rotateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void armJointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void gripperCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void timerCallback();

    RoverDashboard *dashboard_;
    std::vector<double> pivot_drive_;
    std::vector<double> pivot_rotate_;
    std::vector<double> arm_joints_;
    std::vector<double> arm_gripper_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rotate_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_joints_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // ROVER_GUI_NODE_HPP
