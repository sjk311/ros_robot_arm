#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

// 서비스 헤더
#include "robot_arm_gripper_service/srv/move_arm.hpp"
#include "robot_arm_gripper_service/srv/control_gripper.hpp"
#include "robot_arm_gripper_service/srv/rotate_gripper.hpp"

using namespace std::chrono_literals;

class RobotArmGripperServiceNode : public rclcpp::Node
{
public:
  RobotArmGripperServiceNode()
  : Node("robot_arm_gripper_service_node")
  {
    move_arm_service_ = this->create_service<robot_arm_gripper_service::srv::MoveArm>(
      "move_arm", std::bind(&RobotArmGripperServiceNode::handle_move_arm, this, std::placeholders::_1, std::placeholders::_2));

    control_gripper_service_ = this->create_service<robot_arm_gripper_service::srv::ControlGripper>(
      "control_gripper", std::bind(&RobotArmGripperServiceNode::handle_control_gripper, this, std::placeholders::_1, std::placeholders::_2));

    rotate_gripper_service_ = this->create_service<robot_arm_gripper_service::srv::RotateGripper>(
      "rotate_gripper", std::bind(&RobotArmGripperServiceNode::handle_rotate_gripper, this, std::placeholders::_1, std::placeholders::_2));

    // Dynamixel 초기화
    portHandler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (portHandler_->openPort() && portHandler_->setBaudRate(57600)) {
      RCLCPP_INFO(this->get_logger(), "Dynamixel 포트 열기 성공");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Dynamixel 포트 열기 실패");
    }

    RCLCPP_INFO(this->get_logger(), "서비스 노드 시작됨");
  }

private:
  // 역기구학 계산 함수
  bool calculate_inverse_kinematics(float x, float y, float &theta1_deg, float &theta2_deg) {
    float L1 = 20.0;
    float L2 = 16.0;
    float D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (std::abs(D) > 1.0) return false;

    float theta2_rad = std::atan2(std::sqrt(1 - D * D), D); // elbow-down
    float theta1_rad = std::atan2(y, x) - std::atan2(L2 * std::sin(theta2_rad), L1 + L2 * std::cos(theta2_rad));

    theta1_deg = theta1_rad * 180.0 / M_PI;
    theta2_deg = theta2_rad * 180.0 / M_PI;
    return true;
  }

  // 위치 → 포지션 값 변환
  int angle_to_position_joint1(float theta_deg) {
    return static_cast<int>(2048 - theta_deg * (1024.0 / 90.0));
  }
  int angle_to_position_joint2(float theta_deg) {
    return static_cast<int>(2048 - theta_deg * (2048.0 / 180.0));
  }
  int angle_to_position_rotate(float theta_deg) {
    return static_cast<int>(2048 + theta_deg * (2048.0 / 180.0));
  }

  // 목표 위치 전송
  void write_goal_position(int id, int position) {
    int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, 116, position, 0);
    if (dxl_comm_result != COMM_SUCCESS)
      RCLCPP_WARN(this->get_logger(), "모터 %d 위치 전송 실패: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
  }

  // === 서비스 콜백 ===
  void handle_move_arm(
    const std::shared_ptr<robot_arm_gripper_service::srv::MoveArm::Request> request,
    std::shared_ptr<robot_arm_gripper_service::srv::MoveArm::Response> response)
  {
    float theta1, theta2;
    if (!calculate_inverse_kinematics(request->x, request->y, theta1, theta2)) {
      response->success = false;
      response->message = "도달 불가능한 위치입니다.";
      return;
    }

    int pos1 = angle_to_position_joint1(theta1);
    int pos2 = angle_to_position_joint2(theta2);
    write_goal_position(0, pos1);
    write_goal_position(1, pos2);

    response->success = true;
    response->message = "✅ move_arm 실행 완료";
    RCLCPP_INFO(this->get_logger(), "[move_arm] θ1=%.2f°, θ2=%.2f° → pos1=%d, pos2=%d", theta1, theta2, pos1, pos2);
  }

  void handle_control_gripper(
    const std::shared_ptr<robot_arm_gripper_service::srv::ControlGripper::Request> request,
    std::shared_ptr<robot_arm_gripper_service::srv::ControlGripper::Response> response)
  {
    int target_pos = (request->command == "open") ? 3800 : 2500;
    write_goal_position(4, target_pos);

    response->success = true;
    response->message = "✅ control_gripper 실행 완료";
    RCLCPP_INFO(this->get_logger(), "[gripper] command=%s → pos=%d", request->command.c_str(), target_pos);
  }

  void handle_rotate_gripper(
    const std::shared_ptr<robot_arm_gripper_service::srv::RotateGripper::Request> request,
    std::shared_ptr<robot_arm_gripper_service::srv::RotateGripper::Response> response)
  {
    int rotate_pos = angle_to_position_rotate(request->angle_deg);
    write_goal_position(3, rotate_pos);

    response->success = true;
    response->message = "✅ rotate_gripper 실행 완료";
    RCLCPP_INFO(this->get_logger(), "[rotate] angle=%.2f° → pos=%d", request->angle_deg, rotate_pos);
  }

  // 서비스 서버
  rclcpp::Service<robot_arm_gripper_service::srv::MoveArm>::SharedPtr move_arm_service_;
  rclcpp::Service<robot_arm_gripper_service::srv::ControlGripper>::SharedPtr control_gripper_service_;
  rclcpp::Service<robot_arm_gripper_service::srv::RotateGripper>::SharedPtr rotate_gripper_service_;

  // 다이나믹셀 핸들러
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotArmGripperServiceNode>());
  rclcpp::shutdown();
  return 0;
}

