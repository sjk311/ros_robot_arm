// robot_arm_gripper_node.cpp
#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

// === 링크 길이 (단위: cm) ===
const double L1 = 20.0;
const double L2 = 16.0;

// === 다이나믹셀 주소값 ===
const uint16_t ADDR_OPERATING_MODE = 11;
const uint16_t ADDR_TORQUE_ENABLE = 64;
const uint16_t ADDR_GOAL_POSITION = 116;
const uint16_t ADDR_ACCELERATION = 108;
const uint16_t ADDR_VELOCITY = 112;
const uint16_t ADDR_PROFILE_VELOCITY = 112;
const uint16_t ADDR_PRESENT_POSITION = 132;
const uint16_t ADDR_PRESENT_CURRENT = 126;

// === 파라미터 ===
const uint8_t TORQUE_ENABLE = 1;
const int INIT_POSITION = 2048;
const int SLOW_SPEED = 120;
const int OPEN_POSITION = 3800;
const int CLOSE_POSITION = 2500;
const int CURRENT_THRESHOLD = 400;
const double DELAY_MS = 0.05;

int angle_to_position_joint1(double theta_deg) {
    return static_cast<int>(2048 - theta_deg * (1024.0 / 90.0));
}

int angle_to_position_joint2(double theta_deg) {
    if (theta_deg < 180.0)
        return static_cast<int>(2048 - theta_deg * (2048.0 / 180.0));
    else
        return static_cast<int>(2048 + (theta_deg - 180.0) * (2048.0 / 180.0));
}

int angle_to_position_joint3(double theta_deg) {
    return static_cast<int>(2048 + theta_deg * (1024.0 / 90.0));
}

int16_t to_signed(uint16_t val) {
    if (val & (1 << 15)) return val - (1 << 16);
    return val;
}

bool inverse_kinematics(double x_input, double y_input, double &theta1, double &theta2, double &theta3) {
    double x = abs(x_input), y = abs(y_input);
    double D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (abs(D) > 1) return false;

    theta2 = 180.0 - acos(D) * 180.0 / M_PI;
    theta1 = atan2(y, x) - atan2(L2 * sin(M_PI * theta2 / 180.0), L1 + L2 * cos(M_PI * theta2 / 180.0));
    theta1 *= 180.0 / M_PI;
    theta3 = theta2 - theta1;

    if (x_input < 0) {
        theta1 *= -1;
        theta2 = 180.0 + theta2;
        theta3 = 180.0 - theta3;
    }

    return (-90.0 <= theta1 && theta1 <= 90.0);
}

int main() {
    const char* device_name = "/dev/ttyUSB0";
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(device_name);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!portHandler->openPort()) {
        cerr << "포트 열기 실패" << endl;
        return 1;
    }
    if (!portHandler->setBaudRate(57600)) {
        cerr << "보레이트 설정 실패" << endl;
        return 1;
    }

    for (int id : {0, 1, 3}) {
        packetHandler->write4ByteTxRx(portHandler, id, ADDR_ACCELERATION, 30);
        packetHandler->write4ByteTxRx(portHandler, id, ADDR_VELOCITY, 100);
        packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
        packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, INIT_POSITION);
    }

    // 그리퍼 초기화 (ID 4)
    packetHandler->write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, 0);
    packetHandler->write1ByteTxRx(portHandler, 4, ADDR_OPERATING_MODE, 5);
    packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PROFILE_VELOCITY, SLOW_SPEED);
    packetHandler->write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, 1);
    packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, OPEN_POSITION);
    this_thread::sleep_for(chrono::milliseconds(1000));

    string cmd;
    while (true) {
        cout << "\n명령 입력 (x,y / open / close / rotate deg / exit): ";
        getline(cin, cmd);

        if (cmd == "exit") break;
        else if (cmd == "open") {
            cout << "그리퍼 열기..." << endl;
            packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, OPEN_POSITION);
        } else if (cmd == "close") {
            cout << "그리퍼 닫기..." << endl;
            packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, CLOSE_POSITION);
            while (true) {
                this_thread::sleep_for(chrono::duration<double>(DELAY_MS));
                uint16_t raw_current;
                packetHandler->read2ByteTxRx(portHandler, 4, ADDR_PRESENT_CURRENT, &raw_current);
                int16_t current = to_signed(raw_current);
                uint32_t pos;
                packetHandler->read4ByteTxRx(portHandler, 4, ADDR_PRESENT_POSITION, &pos);
                cout << "전류값: " << current << endl;
                if (abs(current) > CURRENT_THRESHOLD) {
                    cout << " 물체 감지됨 → 정지" << endl;
                    packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, pos);
                    break;
                }
                if (pos < CLOSE_POSITION + 50) break;
            }
        } else if (cmd.rfind("rotate", 0) == 0) {
            try {
                double delta;
                delta = stod(cmd.substr(7));
                uint32_t pos;
                packetHandler->read4ByteTxRx(portHandler, 3, ADDR_PRESENT_POSITION, &pos);
                double current_deg = (pos - 2048) * (90.0 / 1024);
                double target_deg = current_deg + delta;
                int target_pos = angle_to_position_joint3(target_deg);
                packetHandler->write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, target_pos);
                cout << "그리퍼 회전: 현재 " << current_deg << "° → 목표 " << target_deg << "°" << endl;
            } catch (...) {
                cout << "사용법: rotate 30 (도 단위)" << endl;
            }
        } else {
            try {
                size_t comma = cmd.find(',');
                double x = stod(cmd.substr(0, comma));
                double y = stod(cmd.substr(comma + 1));
                double t1, t2, t3;
                if (!inverse_kinematics(x, y, t1, t2, t3)) {
                    cout << "도달 불가 또는 θ1 범위 초과" << endl;
                    continue;
                }
                int pos1 = angle_to_position_joint1(t1);
                int pos2 = angle_to_position_joint2(t2);
                int pos3 = angle_to_position_joint3(t3);

                packetHandler->write4ByteTxRx(portHandler, 0, ADDR_GOAL_POSITION, pos1);
                packetHandler->write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, pos2);
                packetHandler->write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, pos3);

                cout << "\n입력 좌표: (" << x << ", " << y << ")\n"
                     << " → θ1 = " << t1 << "°, θ2 = " << t2 << "°, θ3 = " << t3 << "°\n"
                     << " → pos1 = " << pos1 << ", pos2 = " << pos2 << ", pos3 = " << pos3 << endl;
            } catch (...) {
                cout << "입력 오류. 예: 10,5 또는 rotate 30" << endl;
            }
        }
    }

    portHandler->closePort();
    return 0;
}

