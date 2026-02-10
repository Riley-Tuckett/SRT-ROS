cmake_minimum_required(VERSION 3.8)
project(rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Enable Qt MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Include directories
include_directories(include)

# Create executables
add_executable(controller_node src/control_node_controller.cpp)
add_executable(joystick_node src/control_node_joystick.cpp)
add_executable(gui_node 
  src/gui_node_main.cpp
  src/rover_dashboard.cpp
  src/rover_visualization.cpp
  src/rover_gui_node.cpp
)

ament_target_dependencies(controller_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(joystick_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(gui_node
  rclcpp
  std_msgs
)

target_link_libraries(gui_node
  Qt5::Core
  Qt5::Widgets
)

# Install targets
install(TARGETS
  controller_node
  joystick_node
  gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()