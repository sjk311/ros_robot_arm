cmake_minimum_required(VERSION 3.8)
project(robot_arm_gripper_service)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# .srv 파일 포함
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveArm.srv"
  "srv/ControlGripper.srv"
  "srv/RotateGripper.srv"
)

# 실행 파일 빌드
add_executable(robot_arm_gripper_service_node src/robot_arm_gripper_service_node.cpp)
ament_target_dependencies(robot_arm_gripper_service_node
  rclcpp
  dynamixel_sdk
)

# 인터페이스 타입 연결
rosidl_target_interfaces(robot_arm_gripper_service_node
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# 설치
install(TARGETS
  robot_arm_gripper_service_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

