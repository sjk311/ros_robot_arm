#include "robot_arm_control/robot_arm.hpp"
#include <iostream>

RobotArm::RobotArm(const std::string& device_name, int baudrate)
: device_name_(device_name), baudrate_(baudrate)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);  // Protocol 2.0
}

RobotArm::~RobotArm()
{
  portHandler_->closePort();
}

bool RobotArm::initMotors()
{
  if (!portHandler_->openPort()) {
    std::cerr << "Failed to open port!" << std::endl;
    return false;
  }

  if (!portHandler_->setBaudRate(baudrate_)) {
    std::cerr << "Failed to set baudrate!" << std::endl;
    return false;
  }

  std::cout << "Port opened and baudrate set successfully." << std::endl;
  return true;
}

bool RobotArm::setJointPosition(uint8_t id, uint16_t position)
{
  int dxl_comm_result = packetHandler_->write4ByteTxRx(
    portHandler_, id, PRO_ADDR_GOAL_POSITION, position);

  if (dxl_comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to write goal position for ID " << int(id) << std::endl;
    return false;
  }

  return true;
}

std::vector<uint16_t> RobotArm::getJointPositions()
{
  std::vector<uint16_t> positions;

  for (uint8_t id : joint_ids_) {
    uint32_t pos = 0;
    int dxl_comm_result = packetHandler_->read4ByteTxRx(
      portHandler_, id, PRO_ADDR_PRESENT_POSITION, &pos);

    if (dxl_comm_result != COMM_SUCCESS) {
      std::cerr << "Failed to read position for ID " << int(id) << std::endl;
      positions.push_back(0);
    } else {
      positions.push_back(static_cast<uint16_t>(pos));
    }
  }

  return positions;
}


