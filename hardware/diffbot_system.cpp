// Copyright 2021 ros2_control Development Team
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

#include "ros2_ddsm115/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_ddsm115
{
hardware_interface::CallbackReturn Ros2DDSM115Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

 
  
  cfg_.steering_left_name = info_.hardware_parameters["steering_left_name"];
  cfg_.steering_right_name = info_.hardware_parameters["steering_right_name"];
  cfg_.mcu_device = info_.hardware_parameters["mcu_device"];
  cfg_.mcu_baud_rate = std::stoi(info_.hardware_parameters["mcu_baud_rate"]);
  cfg_.mcu_timeout_ms = std::stoi(info_.hardware_parameters["mcu_timeout_ms"]);
  cfg_.mcu_left_servo_id = std::stoi(info_.hardware_parameters["mcu_left_servo_id"]);
  cfg_.mcu_right_servo_id = std::stoi(info_.hardware_parameters["mcu_right_servo_id"]);
  cfg_.mcu_left_servo_offset = std::stoi(info_.hardware_parameters["mcu_left_servo_offset"]);
  cfg_.mcu_right_servo_offset = std::stoi(info_.hardware_parameters["mcu_right_servo_offset"]);
  cfg_.servo_scaler = std::stof(info_.hardware_parameters["servo_scaler"]);
  cfg_.servo_max_angle = std::stof(info_.hardware_parameters["servo_max_angle"]);

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.ddsm115_device = info_.hardware_parameters["ddsm115_device"];
  cfg_.ddsm115_baud_rate = std::stoi(info_.hardware_parameters["ddsm115_baud_rate"]);
  cfg_.ddsm115_timeout_ms = std::stoi(info_.hardware_parameters["ddsm115_timeout_ms"]);
  cfg_.ddsm115_left_wheel_id = std::stoi(info_.hardware_parameters["ddsm115_left_wheel_id"]);
  cfg_.ddsm115_right_wheel_id = std::stoi(info_.hardware_parameters["ddsm115_right_wheel_id"]);

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.ddsm115_left_wheel_id);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.ddsm115_right_wheel_id);
  // steer_l_.setup(cfg_.steering_left_name, cfg_.mcu_left_servo_id);
  // steer_r_.setup(cfg_.steering_right_name, cfg_.mcu_right_servo_id);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Ros2DDSM115Hardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY || hardware_interface::HW_IF_POSITION)
    // {
    //   RCLCPP_FATAL(
    //     rclcpp::get_logger("Ros2DDSM115Hardware"),
    //     "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
    //     joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // if (joint.state_interfaces.size() >= 1)
    // {
    //   RCLCPP_FATAL(
    //     rclcpp::get_logger("Ros2DDSM115Hardware"),
    //     "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
    //     joint.state_interfaces.size());
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    // {
    //   RCLCPP_FATAL(
    //     rclcpp::get_logger("Ros2DDSM115Hardware"),
    //     "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
    //     joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    // {
    //   RCLCPP_FATAL(
    //     rclcpp::get_logger("Ros2DDSM115Hardware"),
    //     "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
    //     joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Ros2DDSM115Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   steer_l_.name, hardware_interface::HW_IF_POSITION, &steer_l_.pos));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   steer_l_.name, hardware_interface::HW_IF_VELOCITY, &steer_l_.vel));
  
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   steer_r_.name, hardware_interface::HW_IF_POSITION, &steer_r_.pos));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //   steer_r_.name, hardware_interface::HW_IF_VELOCITY, &steer_r_.vel));

  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Ros2DDSM115Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //   steer_l_.name, hardware_interface::HW_IF_POSITION, &steer_l_.cmd));

  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //   steer_r_.name, hardware_interface::HW_IF_POSITION, &steer_r_.cmd));

  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn Ros2DDSM115Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Ros2DDSM115Hardware"), "Activating ...please wait...");

  // mcu_comms_.connect(cfg_.mcu_device, cfg_.mcu_baud_rate, cfg_.mcu_timeout_ms);
  ddsm115_comms_.connect(cfg_.ddsm115_device, cfg_.ddsm115_timeout_ms);

  ddsm115_comms_.set_ddsm115_mode(wheel_l_.id, VELOCITY_LOOP);
  ddsm115_comms_.set_ddsm115_mode(wheel_r_.id, VELOCITY_LOOP);

  RCLCPP_INFO(rclcpp::get_logger("Ros2DDSM115Hardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2DDSM115Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Ros2DDSM115Hardware"), "Deactivating ...please wait...");

  // mcu_comms_.disconnect();
  ddsm115_comms_.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("Ros2DDSM115Hardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ros2DDSM115Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Get Servo/Steering Positions and Velocities
  double delta_seconds = period.seconds();

  // double pos_prev = steer_l_.pos;
  // steer_l_.pos = steer_l_.degrees_to_radians(mcu_comms_.get_servo_position());
  // steer_l_.vel = (steer_l_.pos - pos_prev) / delta_seconds;

  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "SL Position is: %f", steer_l_.radians_to_degrees(steer_l_.pos) );
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "SL Velocity is: %f", steer_l_.vel);

  // pos_prev = wheel_r_.pos;
  // steer_r_.pos = steer_r_.degrees_to_radians(mcu_comms_.get_servo_position());
  // steer_r_.vel = (steer_r_.pos - pos_prev) / delta_seconds;
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "SR Position is: %f", steer_r_.pos);
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "SR Velocity is: %f", steer_r_.vel);



  // Get DDSM115 Wheel Positions and Velocities
  ddsm115_comms_.get_ddsm115_mode(wheel_l_.id);
  wheel_l_.pos = -(wheel_l_.degrees_to_radians(ddsm115_comms_.responseData.angle));
  wheel_l_.vel = wheel_l_.rpm_to_rad_per_sec(ddsm115_comms_.responseData.velocity);
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "WL Position is: %f", wheel_l_.pos);
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "WL Velocity is: %f", wheel_l_.vel);

  ddsm115_comms_.get_ddsm115_mode(wheel_r_.id);
  wheel_r_.pos = wheel_r_.degrees_to_radians(ddsm115_comms_.responseData.angle);
  wheel_r_.vel = wheel_r_.rpm_to_rad_per_sec(ddsm115_comms_.responseData.velocity);
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "WR Position is: %f", wheel_r_.pos);
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "WR Velocity is: %f", wheel_r_.vel);


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_ddsm115 ::Ros2DDSM115Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Set SERVO/Steering Positions
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveDDSM115Hardware"), "WL Command is: %f", steer_l_.cmd );
  // mcu_comms_.set_servo_position(steer_l_.id, steer_l_.radians_to_degrees(steer_l_.cmd)/cfg_.servo_scaler + (90 + cfg_.mcu_left_servo_offset) );
  // mcu_comms_.set_servo_position(steer_r_.id, steer_r_.radians_to_degrees(steer_r_.cmd)/cfg_.servo_scaler + (90 + cfg_.mcu_right_servo_offset) );


  // Set DDSM115 Wheel Velocities
  ddsm115_comms_.set_ddsm115_velocity(wheel_l_.id, wheel_l_.cmd * 5, 3);
  ddsm115_comms_.set_ddsm115_velocity(wheel_r_.id, -wheel_r_.cmd * 5, 3);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_ddsm115

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_ddsm115::Ros2DDSM115Hardware, hardware_interface::SystemInterface)
