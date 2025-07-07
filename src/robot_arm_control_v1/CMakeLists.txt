cmake_minimum_required(VERSION 3.8)
project(robot_arm_control_v1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)  # ✅ Dynamixel SDK 추가

# include headers
include_directories(include)

# build: coordinate_input_node
add_executable(coordinate_input_node src/coordinate_input_node.cpp)
ament_target_dependencies(coordinate_input_node
  rclcpp
  geometry_msgs
)

# build: robot_arm_node
add_executable(robot_arm_node
  src/robot_arm_node.cpp
  src/robot_arm.cpp
)
target_include_directories(robot_arm_node PUBLIC include)
ament_target_dependencies(robot_arm_node
  rclcpp
  geometry_msgs
  sensor_msgs
  dynamixel_sdk  # ✅ Dynamixel SDK 연결
)

# install targets
install(TARGETS
  coordinate_input_node
  robot_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# test (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

