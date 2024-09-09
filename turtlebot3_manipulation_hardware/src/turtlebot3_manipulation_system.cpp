// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "turtlebot3_manipulation_hardware/turtlebot3_manipulation_system.hpp"

const uint16_t TORQUE_ENABLE_ADDR = 64;
const uint16_t WRITE_PROFILE_ACCELERATION_ADDR =
    108;                                    // Address for profile acceleration
const uint16_t WRITE_PROFILE_VELOCITY_ADDR = 112; // Address for profile velocity
const int READ_PRESENT_POSITION_REGISTER = 132;
const int READ_PRESENT_VEL_REGISTER = 128;
const int WRITE_GOAL_POSITION_REGISTER = 116;

inline int32_t convert_radian_to_tick(
  const double & radian,
  const int32_t & max_tick,
  const int32_t & min_tick,
  const double & max_radian,
  const double & min_radian)
{
  int32_t tick = 0;
  int32_t zero_tick = (max_tick + min_tick) / 2;

  if (radian > 0) {
    tick = (radian * (max_tick - zero_tick) / max_radian) + zero_tick;
  } else if (radian < 0) {
    tick = (radian * (min_tick - zero_tick) / min_radian) + zero_tick;
  } else {
    tick = zero_tick;
  }

  return tick;
}

inline double convert_tick_to_radian(
  const int32_t & tick,
  const int32_t & max_tick,
  const int32_t & min_tick,
  const double & max_radian,
  const double & min_radian)
{
  double radian = 0.0;
  int32_t zero_tick = (max_tick + min_tick) / 2;

  if (tick > zero_tick) {
    radian = static_cast<double>(tick - zero_tick) * max_radian /
      static_cast<double>(max_tick - zero_tick);
  } else if (tick < zero_tick) {
    radian = static_cast<double>(tick - zero_tick) * min_radian /
      static_cast<double>(min_tick - zero_tick);
  } else {
    radian = 0.0;
  }

  return radian;
}

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
auto logger = rclcpp::get_logger("turtlebot3_manipulation");
hardware_interface::CallbackReturn TurtleBot3ManipulationSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  id_ = stoi(info_.hardware_parameters["opencr_id"]);
  usb_port_ = info_.hardware_parameters["opencr_usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["opencr_baud_rate"]);
  heartbeat_ = 0;

  joints_acceleration_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);

  joints_velocity_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);

  gripper_acceleration_ = stoi(info_.hardware_parameters["dxl_gripper_profile_acceleration"]);
  gripper_velocity_ = stoi(info_.hardware_parameters["dxl_gripper_profile_velocity"]);

#if 0
  opencr_ = std::make_unique<OpenCR>(id_);
  if (opencr_->open_port(usb_port_)) {
    RCLCPP_INFO(logger, "Succeeded to open port");
  } else {
    RCLCPP_FATAL(logger, "Failed to open port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (opencr_->set_baud_rate(baud_rate_)) {
    RCLCPP_INFO(logger, "Succeeded to set baudrate");
  } else {
    RCLCPP_FATAL(logger, "Failed to set baudrate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  int32_t model_number = opencr_->ping();
  RCLCPP_INFO(logger, "OpenCR Model Number %d", model_number);

  if (opencr_->is_connect_manipulator()) {
    RCLCPP_INFO(logger, "Connected manipulator");
  } else {
    RCLCPP_FATAL(logger, "Not connected manipulator");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (opencr_->is_connect_wheels()) {
    RCLCPP_INFO(logger, "Connected wheels");
  } else {
    RCLCPP_FATAL(logger, "Not connected wheels");
    return hardware_interface::CallbackReturn::ERROR;
  }
#endif

    joints_id_[0] = 11;
    joints_id_[1] = 12;
    joints_id_[2] = 13;
    joints_id_[3] = 14;
    gripper_id_ = 15;

    port_handler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (port_handler_->openPort()) {
      RCLCPP_INFO(logger, "Succeeded to open port");
    } else {
      RCLCPP_FATAL(logger, "Failed to open port..... %s\n", usb_port_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (port_handler_->setBaudRate(baud_rate_)) {
      RCLCPP_INFO(logger, "Succeeded to set baudrate");
    } else {
      RCLCPP_FATAL(logger, "Failed to set baudrate");
      return hardware_interface::CallbackReturn::ERROR;
    }

    /* ping */
    for (uint8_t i = 0; i < joints_id_.size(); i++) {
      if (!ping(joints_id_[i])) {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!ping(gripper_id_)) {
      return hardware_interface::CallbackReturn::ERROR;
    }

  dxl_wheel_commands_.resize(2, 0.0);

  dxl_joint_commands_.resize(4, 0.0);
  dxl_joint_commands_[0] = 0.0;
  dxl_joint_commands_[1] = -1.53;
  dxl_joint_commands_[2] = 1.012;
  dxl_joint_commands_[3] = 0.296;

  dxl_gripper_commands_.resize(2, 0.0);

  dxl_positions_.resize(info_.joints.size(), 0.0);
  dxl_velocities_.resize(info_.joints.size(), 0.0);

#if 0
  opencr_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(),
    info_.sensors[1].state_interfaces.size(),
    0.0);
#endif
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TurtleBot3ManipulationSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint8_t i = 0; i < info_.joints.size(); i++) {
    RCLCPP_INFO(logger, "%s", info_.joints[i].name.c_str());
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_velocities_[i]));
  }

#if 0
  for (uint8_t i = 0, k = 0; i < info_.sensors.size(); i++) {
    for (uint8_t j = 0; j < info_.sensors[i].state_interfaces.size(); j++) {
      RCLCPP_ERROR(logger, "%s %s", info_.sensors[i].name.c_str(), info_.sensors[i].state_interfaces[j].name.c_str());
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[i].name,
          info_.sensors[i].state_interfaces[j].name,
          &opencr_sensor_states_[k++])
      );
    }
  }
#endif
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TurtleBot3ManipulationSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

#if 0
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &dxl_wheel_commands_[1]));
#endif
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[2].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[3].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[3]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[4].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[5].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[1]));

  return command_interfaces;
}

hardware_interface::CallbackReturn TurtleBot3ManipulationSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for start");

#if 0
  opencr_->send_heartbeat(heartbeat_++);

  RCLCPP_INFO(logger, "Wait for IMU re-calibration");
  opencr_->imu_recalibration();
  rclcpp::sleep_for(std::chrono::seconds(3));

  RCLCPP_INFO(logger, "Joints and wheels torque ON");
  opencr_->joints_torque(opencr::ON);
  opencr_->wheels_torque(opencr::ON);

  opencr_->send_heartbeat(heartbeat_++);
  RCLCPP_INFO(logger, "Set profile acceleration and velocity to joints");
  opencr_->set_joint_profile_acceleration(joints_acceleration_);
  opencr_->set_joint_profile_velocity(joints_velocity_);

  RCLCPP_INFO(logger, "Set profile acceleration and velocity to gripper");
  opencr_->set_gripper_profile_acceleration(gripper_acceleration_);
  opencr_->set_gripper_profile_velocity(gripper_velocity_);

  RCLCPP_INFO(logger, "Set goal current value to gripper");
  opencr_->set_gripper_current();

  RCLCPP_INFO(logger, "System starting");
  opencr_->play_sound(opencr::SOUND::ASCENDING);
#endif

  RCLCPP_INFO(logger, "Joints and Gripper torque ON");
  for (uint8_t i = 0; i < joints_id_.size(); i++) {
    if (!enable_torque(joints_id_[i], true)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!enable_torque(gripper_id_, true)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger, "Set profile acceleration and velocity to joints");
  for (uint8_t i = 0; i < joints_id_.size(); i++) {
    if (!set_profile_acceleration(joints_id_[i], joints_acceleration_[i])) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!set_profile_velocity(joints_id_[i], joints_velocity_[i])) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (!set_profile_acceleration(gripper_id_, gripper_acceleration_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!set_profile_velocity(gripper_id_, gripper_velocity_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger, "Set goal current value to gripper");
  //set_gripper_current(gripper_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurtleBot3ManipulationSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for stop");
  //opencr_->play_sound(opencr::SOUND::DESCENDING);
  std::array<int32_t, 4> tick = {2048, 750, 3040, 2500};
  for (uint8_t i = 0; i < joints_id_.size(); i++) {
    auto rad = convert_tick_to_radian(tick[i], opencr::joints::MAX_TICK,
    opencr::joints::MIN_TICK,
    opencr::joints::MAX_RADIAN,
    opencr::joints::MIN_RADIAN);
    set_joint_position(joints_id_[i], rad);
  }

  set_gripper_position(gripper_id_, 0.0);

  RCLCPP_INFO(logger, "System stopped. Disabling torque");
  sleep(3);

  for (uint8_t i = 0; i < joints_id_.size(); i++) {
    enable_torque(joints_id_[i], false);
  }
  enable_torque(gripper_id_, false);
  RCLCPP_INFO(logger, "Goodbye...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to read wheels and manipulator states");

#if 0
  if (opencr_->read_all() == false) {
    RCLCPP_WARN(logger, "Failed to read all control table");
  }

  dxl_positions_[0] = opencr_->get_wheel_positions()[opencr::wheels::LEFT];
  dxl_velocities_[0] = opencr_->get_wheel_velocities()[opencr::wheels::LEFT];

  dxl_positions_[1] = opencr_->get_wheel_positions()[opencr::wheels::RIGHT];
  dxl_velocities_[1] = opencr_->get_wheel_velocities()[opencr::wheels::RIGHT];
#endif

  read_joint_position(joints_id_[0], dxl_positions_[0]);
  read_joint_velocity(joints_id_[0], dxl_velocities_[0]);

  read_joint_position(joints_id_[1], dxl_positions_[1]);
  read_joint_velocity(joints_id_[1], dxl_velocities_[1]);

  read_joint_position(joints_id_[2], dxl_positions_[2]);
  read_joint_velocity(joints_id_[2], dxl_velocities_[2]);

  read_joint_position(joints_id_[3], dxl_positions_[3]);
  read_joint_velocity(joints_id_[3], dxl_velocities_[3]);

  read_gripper_position(gripper_id_, dxl_positions_[4]);
  read_gripper_velocity(gripper_id_, dxl_velocities_[4]);

  read_gripper_position(gripper_id_, dxl_positions_[5]);
  read_gripper_velocity(gripper_id_, dxl_velocities_[5]);
#if 0
  opencr_sensor_states_[0] = opencr_->get_imu().orientation.x;
  opencr_sensor_states_[1] = opencr_->get_imu().orientation.y;
  opencr_sensor_states_[2] = opencr_->get_imu().orientation.z;
  opencr_sensor_states_[3] = opencr_->get_imu().orientation.w;

  opencr_sensor_states_[4] = opencr_->get_imu().angular_velocity.x;
  opencr_sensor_states_[5] = opencr_->get_imu().angular_velocity.y;
  opencr_sensor_states_[6] = opencr_->get_imu().angular_velocity.z;

  opencr_sensor_states_[7] = opencr_->get_imu().linear_acceleration.x;
  opencr_sensor_states_[8] = opencr_->get_imu().linear_acceleration.y;
  opencr_sensor_states_[9] = opencr_->get_imu().linear_acceleration.z;

  opencr_sensor_states_[10] = opencr_->get_battery().voltage;
  opencr_sensor_states_[11] = opencr_->get_battery().percentage;
  opencr_sensor_states_[12] = opencr_->get_battery().design_capacity;
  opencr_sensor_states_[13] = opencr_->get_battery().present;
#endif
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtleBot3ManipulationSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to write wheels and manipulator commands");
  #if 0
  opencr_->send_heartbeat(heartbeat_++);

  if (opencr_->set_wheel_velocities(dxl_wheel_commands_) == false) {
    RCLCPP_ERROR(logger, "Can't control wheels");
  }

  if (opencr_->set_joint_positions(dxl_joint_commands_) == false) {
    RCLCPP_ERROR(logger, "Can't control joints");
  }
  #endif

  for (uint8_t i = 0; i < joints_id_.size(); i++) {
    set_joint_position(joints_id_[i], dxl_joint_commands_[i]);
  }

  set_gripper_position(gripper_id_, dxl_gripper_commands_[0]);
  return hardware_interface::return_type::OK;
}

bool TurtleBot3ManipulationSystemHardware::ping(uint8_t id) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);

  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result
  uint16_t model_number = 0;

  dxl_comm_result = packet_handler_->ping(port_handler_, id, &model_number, &dxl_error);

  // Check the result of the ping operation
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Ping Failed! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  RCLCPP_ERROR(logger,"[%d] Ping Succeeded! model_number[%d]\n", id, model_number);

  return true;
}

bool TurtleBot3ManipulationSystemHardware::enable_torque(uint8_t id,
                                                         uint8_t enable) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result

  dxl_comm_result = packet_handler_->write1ByteTxRx(
      port_handler_, id, TORQUE_ENABLE_ADDR, enable, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to enable torque! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  RCLCPP_ERROR(logger,"%d] Torque enabled successfully.\n", id);
  return true;
}
bool TurtleBot3ManipulationSystemHardware::set_profile_acceleration(
    uint8_t id, uint32_t val) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result

  dxl_comm_result = packet_handler_->write4ByteTxRx(
      port_handler_, id, WRITE_PROFILE_ACCELERATION_ADDR, val, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to set_profile_acceleration! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  RCLCPP_ERROR(logger,"[%d] set_profile_acceleration successfully.\n", id);
  return true;
}
bool TurtleBot3ManipulationSystemHardware::set_profile_velocity(uint8_t id,
                                                                uint32_t val) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result

  dxl_comm_result = packet_handler_->write4ByteTxRx(
      port_handler_, id, WRITE_PROFILE_VELOCITY_ADDR, val, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to set_profile_velocity! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  RCLCPP_ERROR(logger,"[%d] set_profile_velocity successfully.\n", id);
  return true;
}

bool TurtleBot3ManipulationSystemHardware::set_joint_position(uint8_t id,
                                                                double val) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result

  int tick = convert_radian_to_tick(
      val,
      opencr::joints::MAX_TICK,
      opencr::joints::MIN_TICK,
      opencr::joints::MAX_RADIAN,
      opencr::joints::MIN_RADIAN);


  dxl_comm_result = packet_handler_->write4ByteTxRx(
      port_handler_, id, WRITE_GOAL_POSITION_REGISTER, tick, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to set_joint_position! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  //RCLCPP_ERROR(logger,"[%d] set_joint_position successfully.\n", id);
  return true;
}

bool TurtleBot3ManipulationSystemHardware::set_gripper_position(uint8_t id,
                                                                double val) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result

  double radian = val / opencr::grippers::RAD_TO_METER;
  int tick = convert_radian_to_tick(
      radian,
      opencr::joints::MAX_TICK,
      opencr::joints::MIN_TICK,
      opencr::joints::MAX_RADIAN,
      opencr::joints::MIN_RADIAN);


  dxl_comm_result = packet_handler_->write4ByteTxRx(
      port_handler_, id, WRITE_GOAL_POSITION_REGISTER, tick, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to set_gripper_position! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  //RCLCPP_ERROR(logger,"[%d] set_gripper_position successfully.\n", id);
  return true;
}

bool TurtleBot3ManipulationSystemHardware::set_gripper_current(uint8_t id)
{
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result

  dxl_comm_result = packet_handler_->write2ByteTxRx(
      port_handler_, id, 102, 1, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to set_gripper_current! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  RCLCPP_ERROR(logger,"[%d] set_gripper_current successfully.\n", id);
  return true;
}


bool TurtleBot3ManipulationSystemHardware::read_joint_velocity(uint8_t id,
                                                                double& val) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result
  int32_t rpm = 0;

  val = 0;
  dxl_comm_result = packet_handler_->read4ByteTxRx(
      port_handler_, id, READ_PRESENT_VEL_REGISTER, (uint32_t*)&rpm, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to read_joint_velocity! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  val = rpm * opencr::joints::RPM_TO_RAD_PER_SEC;
  //RCLCPP_ERROR(logger,"[%d] read_joint_velocity [%f]", id, val);
  return true;
}
bool TurtleBot3ManipulationSystemHardware::read_joint_position(uint8_t id,
                                                                double& val) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result
  int32_t tick = 0;

  val = 0.0;
  dxl_comm_result = packet_handler_->read4ByteTxRx(
      port_handler_, id, READ_PRESENT_POSITION_REGISTER, (uint32_t*)&tick, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to read_joint_position! %s", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  val = convert_tick_to_radian(tick, opencr::joints::MAX_TICK,
    opencr::joints::MIN_TICK,
    opencr::joints::MAX_RADIAN,
    opencr::joints::MIN_RADIAN);
  //RCLCPP_ERROR(logger,"[%d] read_joint_position [%f]", id, val);
  return true;
}
bool TurtleBot3ManipulationSystemHardware::read_gripper_velocity(uint8_t id,
                                                                double& val) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result
  int32_t rpm = 0;

  val = 0;
  dxl_comm_result = packet_handler_->read4ByteTxRx(
      port_handler_, id, READ_PRESENT_VEL_REGISTER, (uint32_t*)&rpm, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to read_joint_velocity! %s\n", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  val = rpm * opencr::joints::RPM_TO_RAD_PER_SEC;
  return true;
}
bool TurtleBot3ManipulationSystemHardware::read_gripper_position(uint8_t id,
                                                                double& val) {
  std::lock_guard<std::mutex> lock(sdk_handler_m_);
  uint8_t dxl_error = 0; // Variable to store error codes
  int dxl_comm_result;   // Variable to store communication result
  int32_t tick = 0;

  val = 0.0;
  dxl_comm_result = packet_handler_->read4ByteTxRx(
      port_handler_, id, READ_PRESENT_POSITION_REGISTER, (uint32_t*)&tick, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(logger,"[%d] Failed to read_joint_position! %s", id,
           packet_handler_->getRxPacketError(dxl_error));
    return false;
  }
  val = convert_tick_to_radian(tick, opencr::joints::MAX_TICK,
    opencr::joints::MIN_TICK,
    opencr::joints::MAX_RADIAN,
    opencr::joints::MIN_RADIAN);
  val *= opencr::grippers::RAD_TO_METER;
  //RCLCPP_ERROR(logger,"[%d] read_joint_position [%f]", id, val);
  return true;
}
} // namespace turtlebot3_manipulation_hardware
} // namespace robotis

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotis::turtlebot3_manipulation_hardware::TurtleBot3ManipulationSystemHardware,
  hardware_interface::SystemInterface)
