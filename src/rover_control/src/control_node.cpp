#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class ControlNode :  public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node")
  {
    // Create subscriber to joy topic
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ControlNode::joy_callback, this, std::placeholders::_1));

    // Create publisher for control commands
    control_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/rover/control_commands", 10);

    // Create timer for 50Hz publishing
    timer_ = this->create_wall_timer(
      20ms,
      std::bind(&ControlNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to:  /joy");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /rover/control_commands");
    RCLCPP_INFO(this->get_logger(), "Publish rate: 50 Hz");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy:: SharedPtr msg)
  {
    // Store the latest joy message
    latest_joy_ = msg;
    
    // Log when we receive joy data (only log first time)
    if (!joy_received_) {
      RCLCPP_INFO(this->get_logger(), "Joy data received!  Axes: %zu, Buttons: %zu", 
                  msg->axes. size(), msg->buttons.size());
      joy_received_ = true;
    }
  }

  void timer_callback()
  {
    // Only publish if we have received joy data
    if (! latest_joy_) {
      return;
    }

    // Create control command message
    auto control_msg = std_msgs::msg::Float32MultiArray();
    
    // TODO: Replace this with Disha's code
    
    // Placeholder mapping:
    // Axes[0] = Left stick X (left/right)
    // Axes[1] = Left stick Y (up/down)
    // Axes[2] = Right stick X
    // Axes[3] = Right stick Y
    
    if (latest_joy_->axes.size() >= 4) {
      // Placeholder:  Extract and pass through the first 4 axes
      float left_x = latest_joy_->axes[0];
      float left_y = latest_joy_->axes[1];
      float right_x = latest_joy_->axes[2];
      float right_y = latest_joy_->axes[3];

      // Make messages
      control_msg. data.push_back(left_x);
      control_msg. data.push_back(left_y);
      control_msg. data.push_back(right_x);
      control_msg. data.push_back(right_y);
      
      // Publish the control commands
      control_publisher_->publish(control_msg);
      
      // Debugging information, comment when unneeded
      RCLCPP_INFO(this->get_logger(), "Publishing: [%.2f, %.2f, %.2f, %.2f]", 
                   left_x, left_y, right_x, right_y);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<std_msgs:: msg::Float32MultiArray>::SharedPtr control_publisher_;
  rclcpp::TimerBase:: SharedPtr timer_;
  
  sensor_msgs::msg::Joy:: SharedPtr latest_joy_;
  bool joy_received_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(rclcpp::get_logger("control_node"), "Starting Control Node.. .");
  
  auto node = std::make_shared<ControlNode>();
  
  rclcpp::spin(node);
  
  rclcpp:: shutdown();
  return 0;
}