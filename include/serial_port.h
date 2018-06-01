#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <SerialStream.h>
#include <string>

using namespace LibSerial;

bool SetupSerialPort(SerialStream &serial_stream,
                     const int baud_rate,
                     std::string &input_serial_dev);

#endif

