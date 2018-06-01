#include "serial_port.h"

using namespace LibSerial;

bool SetupSerialPort(SerialStream &serial_stream,
                     const int baud_rate,  std::string &input_serial_dev) {

  // Open the Serial Port at the desired hardware port.
  input_serial_dev = std::string("/dev/") + input_serial_dev;
  serial_stream.Open(input_serial_dev);
  bool is_speed_set = true;

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
      is_speed_set = false;
      std::cerr << "Baud rate must be 115200, 57600, 38400, 19200, 9600, 4800, 2400 or 1800\n";
      break;
  }

  if (!is_speed_set)
    return is_speed_set;

  // Set the number of data bits.
  serial_stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

  // Turn off hardware flow control.
  serial_stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

  // Disable parity.
  serial_stream.SetParity(Parity::PARITY_NONE);

  // Set the number of stop bits.
  serial_stream.SetStopBits(StopBits::STOP_BITS_1);

  return true;
}

