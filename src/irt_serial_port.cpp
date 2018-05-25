#include "irt_serial_port.h"

using namespace LibSerial;

void SetupSerialPort(SerialStream &serial_stream, const int baud_rate) {

  // Open the Serial Port at the desired hardware port.
  serial_stream.Open("/dev/ttyS0");

  switch (baud_rate) {
    case 115200:
      serial_stream.SetBaudRate(BaudRate::BAUD_115200);
      break;
    case 57600:
      serial_stream.SetBaudRate(BaudRate::BAUD_57600);
      break;
    case 38400:
      serial_stream.SetBaudRate(BaudRate::BAUD_38400);
      break;
    case 19200:
      serial_stream.SetBaudRate(BaudRate::BAUD_19200);
      break;
    case 9600:
      serial_stream.SetBaudRate(BaudRate::BAUD_9600);
      break;
    case 4800:
      serial_stream.SetBaudRate(BaudRate::BAUD_4800);
      break;
    case 2400:
      serial_stream.SetBaudRate(BaudRate::BAUD_2400);
      break;
    case 1800:
      serial_stream.SetBaudRate(BaudRate::BAUD_1800);
      break;
    default:
      serial_stream.SetBaudRate(BaudRate::BAUD_115200);
      break;
  }

  // Set the number of data bits.
  serial_stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

  // Turn off hardware flow control.
  serial_stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

  // Disable parity.
  serial_stream.SetParity(Parity::PARITY_NONE);

  // Set the number of stop bits.
  serial_stream.SetStopBits(StopBits::STOP_BITS_1);
}

