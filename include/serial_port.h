#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <SerialStream.h>

using namespace LibSerial;

bool SetupSerialPort(SerialStream &serial_stream, const int baud_rate = 115200);

#endif

