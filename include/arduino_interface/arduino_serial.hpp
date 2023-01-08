#ifndef ARDUINO_SERIAL_H
#define ARDUINO_SERIAL_H
#include "serial/serial.h"
class ArduinoSerial{
    public:
    void init();
    std::tuple<double, double> read();
    void write_msg(std::string msg);
    private:
    serial::Serial serial_conn;


};
#endif