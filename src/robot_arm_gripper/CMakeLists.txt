cmake_minimum_required(VERSION 3.8)
project(robot_arm_gripper)

# C++ 표준
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# 실행 파일
add_executable(robot_arm_gripper_node src/robot_arm_gripper_node.cpp)
ament_target_dependencies(robot_arm_gripper_node rclcpp dynamixel_sdk)

# 설치
install(TARGETS
  robot_arm_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

