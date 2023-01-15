

#include "arduino_interface/arduino_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arduino_interface
{
const std::string left_wheel = "left_wheel_joint";
const std::string right_wheel = "right_wheel_joint";
double prev_l = 0;
double prev_r = 0;
auto time = std::chrono::system_clock::now();


hardware_interface::return_type ArduinoInterface::configure(const hardware_interface::HardwareInfo & info)
{
  

  
  
  serial.init();
  hw_cmd_left=0;
  hw_cmd_right=0;
  hw_vel_left=0;
  hw_vel_right=0;
  hw_pos_left=0;
  hw_pos_right=0;
  

  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Successfully configured!");

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
ArduinoInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel, hardware_interface::HW_IF_VELOCITY, &hw_vel_left));
  state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel, hardware_interface::HW_IF_POSITION, &hw_pos_left));
  state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel, hardware_interface::HW_IF_VELOCITY, &hw_vel_right));
  state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel, hardware_interface::HW_IF_POSITION, &hw_pos_right));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArduinoInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel, hardware_interface::HW_IF_VELOCITY, &hw_cmd_left));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel, hardware_interface::HW_IF_VELOCITY, &hw_cmd_right));
  
  return command_interfaces;
}



hardware_interface::return_type ArduinoInterface::read()
{
  std::tuple<double, double> vels = serial.read();
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time;
  double deltas = diff.count();
  time = new_time;
  hw_pos_left = std::get<0>(vels);
  hw_pos_right = std::get<1>(vels);
  hw_vel_left =(hw_pos_left-prev_l)/deltas;
  hw_vel_right =(hw_pos_right-prev_r)/deltas;
  prev_l = hw_pos_left;
  prev_r = hw_pos_right;
  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "%f %f", hw_vel_left, hw_vel_right);
  
  
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoInterface::write()
{
  
  float rpm_l = hw_cmd_left*60/(2*M_PI);
  float rpm_r = hw_cmd_right*60/(2*M_PI);
  std::stringstream ss;
  ss<<"w"<<rpm_l<<" "<<rpm_r<<"\n";
  serial.write_msg(ss.str());
  
  


    

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type ArduinoInterface::start(){
  hw_cmd_left = hw_vel_left;
  hw_cmd_right=hw_vel_right;
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type ArduinoInterface::stop(){
  
  return hardware_interface::return_type::OK;
}
}  
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arduino_interface::ArduinoInterface, hardware_interface::SystemInterface)
