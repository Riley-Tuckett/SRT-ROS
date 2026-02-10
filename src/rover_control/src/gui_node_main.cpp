// -----------------------------------------------------------------
// Rover GUI Node - Main Entry Point
// -----------------------------------------------------------------

#include "rover_dashboard.hpp"
#include "rover_gui_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <memory>
#include <thread>

int main(int argc, char *argv[]) {
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Initialize Qt
    QApplication app(argc, argv);

    // Create dashboard
    RoverDashboard dashboard;
    dashboard.show();

    // Create ROS node
    auto node = std::make_shared<RoverGUINode>(&dashboard);

    // Spin ROS in a separate thread
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    // Run Qt event loop
    int result = app.exec();

    // Cleanup
    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    return result;
}
