// Copyright 2020 ros2_control Development Team
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

#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"



using hardware_interface::return_type;
namespace arduino_interface
{
class ArduinoInterface: public hardware_interface::SystemInterface
{
public:
  

  

  
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type start() override;
  return_type stop() override;
  return_type read() override;

  
  return_type write() override;

private:
  
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  
  double hw_vel_left;
  double hw_pos_left;
  double hw_cmd_left;
  double hw_vel_right;
  double hw_pos_right;
  double hw_cmd_right;
  
  ArduinoSerial serial;
};

}  

#endif  
