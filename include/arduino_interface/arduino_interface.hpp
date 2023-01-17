#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_

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
#include "arduino_serial.hpp"


namespace arduino_interface
{
class ArduinoInterface: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoInterface);

  
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  
  hardware_interface::return_type start() override;

 
  hardware_interface::return_type stop() override;

  
  hardware_interface::return_type read() override;

  
  hardware_interface::return_type write() override;

private:
  

  
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