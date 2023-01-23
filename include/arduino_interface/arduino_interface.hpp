

#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "arduino_serial.hpp"

namespace arduino_interface
{
class ArduinoInterface: public hardware_interface::SystemInterface
{
public:
  

  

  
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

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
