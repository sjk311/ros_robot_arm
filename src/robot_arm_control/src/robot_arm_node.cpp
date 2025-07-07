#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_arm_control/robot_arm.hpp"

#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

class RobotArmNode : public rclcpp::Node
{
public:
  RobotArmNode()
  : Node("robot_arm_node")
  {
    // 로봇팔 객체 초기화
    robot_arm_ = std::make_shared<RobotArm>("/dev/ttyUSB0", 57600);
    if (!robot_arm_->initMotors()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize motors.");
      rclcpp::shutdown();
    }

    // joint_states 퍼블리셔 설정
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // 타이머 콜백 주기 설정 (10Hz)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&RobotArmNode::update, this));
  }

private:
  void update()
{
  auto positions = robot_arm_->getJointPositions();

  // 로그 출력
  RCLCPP_INFO(this->get_logger(), "Raw positions: %d, %d", positions[0], positions[1]);

  constexpr double DXL_CENTER = 2048.0;
  constexpr double DXL_RESOLUTION_DEG = 0.088;
  constexpr double DEG2RAD = M_PI / 180.0;

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = {"joint1", "joint2"};
  msg.position.resize(2);
  msg.position[0] = (positions[0] - DXL_CENTER) * DXL_RESOLUTION_DEG * DEG2RAD;
  msg.position[1] = (positions[1] - DXL_CENTER) * DXL_RESOLUTION_DEG * DEG2RAD;

  joint_pub_->publish(msg);
}


  std::shared_ptr<RobotArm> robot_arm_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

