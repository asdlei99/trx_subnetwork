#ifndef IRT_SERIAL_PORT_H
#define IRT_SERIAL_PORT_H

#include <SerialStream.h>

using namespace LibSerial;

void SetupSerialPort(SerialStream &serial_stream, const int baud_rate = 115200);

#endif

