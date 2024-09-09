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

#ifndef TURTLEBOT3_MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_
#define TURTLEBOT3_MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_

#include <memory>
#include <stdbool.h>
#include <string>
#include <vector>
#include <iostream>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "turtlebot3_manipulation_hardware/opencr.hpp"
#include "turtlebot3_manipulation_hardware/visibility_control.h"

namespace robotis
{
namespace turtlebot3_manipulation_hardware
{
class TurtleBot3ManipulationSystemHardware
  : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TurtleBot3ManipulationSystemHardware);

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TURTLEBOT3_MANIPULATION_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;

  std::mutex sdk_handler_m_;
  uint8_t id_;
  std::string usb_port_;
  uint32_t baud_rate_;
  uint8_t heartbeat_;

  std::array<uint8_t, 4> joints_id_;
  uint8_t gripper_id_;

  std::array<int32_t, 4> joints_acceleration_;
  std::array<int32_t, 4> joints_velocity_;

  int32_t gripper_acceleration_;
  int32_t gripper_velocity_;

  //std::unique_ptr<OpenCR> opencr_;

  std::vector<double> dxl_wheel_commands_;
  std::vector<double> dxl_joint_commands_;
  std::vector<double> dxl_gripper_commands_;

  std::vector<double> dxl_positions_;
  std::vector<double> dxl_velocities_;

  std::vector<double> opencr_sensor_states_;

  bool ping(uint8_t id);
  bool enable_torque(uint8_t id, uint8_t enable);
  bool set_profile_acceleration(uint8_t id, uint32_t val);
  bool set_profile_velocity(uint8_t id, uint32_t val);
  bool set_joint_position(uint8_t id, double val);
  bool set_gripper_position(uint8_t id, double val);
  bool read_joint_velocity(uint8_t id, double& val);
  bool read_joint_position(uint8_t id, double& val);
  bool read_gripper_velocity(uint8_t id, double& val);
  bool read_gripper_position(uint8_t id, double& val);
  bool set_gripper_current(uint8_t id);
};
}  // namespace turtlebot3_manipulation_hardware
}  // namespace robotis
#endif  // TURTLEBOT3_MANIPULATION_HARDWARE__TURTLEBOT3_MANIPULATION_SYSTEM_HPP_
