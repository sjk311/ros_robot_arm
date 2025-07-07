#ifndef ROBOT_ARM_CONTROL__ROBOT_ARM_HPP_
#define ROBOT_ARM_CONTROL__ROBOT_ARM_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string>
#include <vector>

class RobotArm
{
public:
  RobotArm(const std::string& device_name, int baudrate);
  ~RobotArm();

  bool initMotors();
  bool setJointPosition(uint8_t id, uint16_t position);  // 0~4095
  std::vector<uint16_t> getJointPositions();             // [joint0, joint1]

private:
  std::string device_name_;
  int baudrate_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  const uint8_t joint_ids_[2] = {0, 1};         // ID 0: base, ID 1: end
  const uint16_t PRO_ADDR_GOAL_POSITION = 116;  // Address for Goal Position
  const uint16_t PRO_ADDR_PRESENT_POSITION = 132;  // Address for Present Position
  const uint16_t LEN_POSITION = 4;              // Length of position value
};

#endif  // ROBOT_ARM_CONTROL__ROBOT_ARM_HPP_

