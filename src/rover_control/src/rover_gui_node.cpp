#include "rover_gui_node.hpp"
#include "rover_dashboard.hpp"
#include <chrono>
#include <functional>

RoverGUINode::RoverGUINode(RoverDashboard *dashboard) 
    : Node("rover_gui"), dashboard_(dashboard) {
    
    // Initialize data
    pivot_drive_ = {0.0, 0.0, 0.0, 0.0};
    pivot_rotate_ = {0.0, 0.0, 0.0, 0.0};
    arm_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    arm_gripper_ = {0.0, 0.0};

    // Create subscriptions
    drive_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "Pivot_Drive", 10,
        std::bind(&RoverGUINode::driveCallback, this, std::placeholders::_1));

    rotate_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "Pivot_Rotate", 10,
        std::bind(&RoverGUINode::rotateCallback, this, std::placeholders::_1));

    arm_joints_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "Arm_Joints", 10,
        std::bind(&RoverGUINode::armJointsCallback, this, std::placeholders::_1));

    gripper_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "Arm_Gripper", 10,
        std::bind(&RoverGUINode::gripperCallback, this, std::placeholders::_1));

    // Timer for updating GUI
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&RoverGUINode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Rover GUI Node initialized");
}

void RoverGUINode::driveCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    pivot_drive_.assign(msg->data.begin(), msg->data.end());
}

void RoverGUINode::rotateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    pivot_rotate_.assign(msg->data.begin(), msg->data.end());
}

void RoverGUINode::armJointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    arm_joints_.assign(msg->data.begin(), msg->data.end());
}

void RoverGUINode::gripperCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    arm_gripper_.assign(msg->data.begin(), msg->data.end());
}

void RoverGUINode::timerCallback() {
    dashboard_->updateDriveData(pivot_drive_, pivot_rotate_);
    dashboard_->updateArmData(arm_joints_, arm_gripper_);
}
