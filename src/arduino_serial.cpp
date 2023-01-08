#include "arduino_interface/arduino_serial.hpp"
#include <tuple>
#include <cmath>
void ArduinoSerial::init(){
    serial_conn.setPort("/dev/ttyACM0");
    serial_conn.setBaudrate(57600);
    serial::Timeout tt = serial::Timeout::simpleTimeout(1000);
    serial_conn.setTimeout(tt); 
    serial_conn.open();
}
std::tuple<double, double> ArduinoSerial::read(){
    
    serial_conn.write("r\n");
    std::string delimiter = " ";
    std::string result = serial_conn.readline();
    size_t del_pos = result.find(delimiter);
    std::string L_VEL = result.substr(0, del_pos);
    std::string R_VEL = result.substr(del_pos + delimiter.length());
    float l_pos = (float)std::atof(L_VEL.c_str());
    float r_pos = (float)std::atof(R_VEL.c_str());
    l_pos = (2*M_PI/200.0)*l_pos;
    r_pos = (2*M_PI/200.0)*r_pos;
    std::tuple<double,double> vels = std::make_tuple(l_pos, r_pos);
    return vels;
    


}
void ArduinoSerial::write_msg(std::string msg){
    serial_conn.write(msg);
}