cmake_minimum_required(VERSION 3.8)
project(robot_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  lib/dynamixel_sdk/include
  lib/dynamixel_sdk/include/dynamixel_sdk
)

add_executable(robot_arm_node
  src/robot_arm_node.cpp
  src/robot_arm.cpp
  lib/dynamixel_sdk/src/dynamixel_sdk/group_bulk_read.cpp
  lib/dynamixel_sdk/src/dynamixel_sdk/group_bulk_write.cpp
  lib/dynamixel_sdk/src/dynamixel_sdk/packet_handler.cpp
  lib/dynamixel_sdk/src/dynamixel_sdk/port_handler_linux.cpp
  lib/dynamixel_sdk/src/dynamixel_sdk/protocol1_packet_handler.cpp
  lib/dynamixel_sdk/src/dynamixel_sdk/protocol2_packet_handler.cpp
  lib/dynamixel_sdk/src/dynamixel_sdk/port_handler.cpp
  lib/dynamixel_sdk/src/dynamixel_sdk/group_handler.cpp
)

ament_target_dependencies(robot_arm_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  robot_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

