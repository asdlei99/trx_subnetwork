#ifndef TRX_SUB_NETWORK_H
#define TRX_SUB_NETWORK_H

#include <cstdint>
#include <cstring>
#include <net/if.h>
#include <cstdlib>
#include <iostream>

#include "serial_port.h"
#include <SerialStream.h>
using namespace LibSerial;

// The frame boundary flag '~' (7E in hexadecimal) marks the beginning and
// end of a frame.
#define FRAME_BOUNDARY_FLAG 0x7E

// A "control escape octet" flag '}' (7D in hexadecimal).
#define CONTROL_ESCAPE_OCTET 0x7D

// If either of the above two octets appear in the transmitted data, an escape
// octet is sent, followed by the original data octet with bit 5 inverted.
#define INVERT_OCTET 0x20

// Maximum length of frame in bytes.
#define MAX_FRAME_LENGTH 2048


// Enum class represents HDLC finite state machine's states for parsing
// received data.
enum class HdlcState {
  START = 0,
  FRAME,
  ESCAPE
};


struct TrxSubNetwork {

  uint32_t frame_position_;
  uint8_t* received_frame_buffer_;

  HdlcState current_state_;

  // Note that currently we use serial port communication between two TRX
  // machines, later is has to be VHF based communication.
  SerialStream serial_stream;

  // TUN file descriptor and device name.
  int tun_fd_;
  char* tun_dev_;


  // Constructor.
  //
  // Arguments:
  // - frame_length: Frame length in bytes, note that in both side it has to
  //                 be the same. By default it is set value of MAX_FRAME_LENGTH.
  //
  // - baud_rate: Serial port speed (baud rate). By default it is set 115200.
  // - tun_dev: TUN device name, by default it is set 'tun0'.
  TrxSubNetwork(const uint16_t frame_size = MAX_FRAME_LENGTH,
                const int baud_rate = 115200, const char* input_tun_dev = "/dev/net/tun0") :
      frame_position_(0),
      received_frame_buffer_(new uint8_t[frame_size + 1]),
      current_state_(HdlcState::START) {

        // Instantiate a SerialStream object.
        bool is_serial_set = SetupSerialPort(serial_stream, baud_rate);
        if (!is_serial_set) {
          delete[] received_frame_buffer_;
          std::cerr << "Failed to setup serial port.\n";
          std::exit(EXIT_FAILURE);
        }

        // Setup TUN interface.
        tun_dev_ = new char[IFNAMSIZ + 1];
        std::strcpy(tun_dev_, input_tun_dev);
        tun_fd_ = TunAlloc(tun_dev_);
        if (tun_fd_ < 0) {
          delete[] tun_dev_;
          delete[] received_frame_buffer_;
          std::cerr << "Failed to create TUN interface.\n";
          std::exit(EXIT_FAILURE);
        }
    };


  // Destructor.
  ~TrxSubNetwork() {
    delete[] tun_dev_;
    delete[] received_frame_buffer_;
  }

  // Function finds valid HDLC frame from incoming data.
  // If frame data received correct including frame flags (byte by byte at a time)
  // it will store frame data to received_frame_buffer_ array and calls
  // HandleFrameData() function, which processes frame.
  //
  // Arguments:
  // - data: Received byte data from serial port.
  void ParseByteData(uint8_t data);

  // Function send a byte data through serial port.
  //
  // Arguments:
  // - data: A data which will be sent.
  void SendByte(char data);

  // Function gets as an input frame data, encodes it to HDLC frame format and
  // sends it out byte at a time using SendByte() function.
  //
  // Arguments:
  // - frame_buffer: Frame data or original frame.
  // - frame_length: Frame length in bytes.
  void FrameEncodeToHdlcAndSend(const uint8_t* frame_buffer, uint32_t frame_length);

  // Function processes received frame, basically writes data to to TUN /dev/net/tun
  //
  // Arguments:
  // - frame_data: Received frame data.
  // - frame_length: A length of frame.
  void HandleFrameData(const uint8_t* frame_data, const uint32_t frame_length);

  // Network device ("/dev/net/tun") allocation.
  //
  // Arguments:
  // - dev: It should be the name of the device with a format string. (e.g. "tun%d").
  //        Note that the character pointer becomes overwritten with the real device
  //        name (e.g. "tun0").
  //
  // Return the file descriptor of the new tun device.
  int TunAlloc(char* dev);

  // Function is running as a separate thread of process and it is always listening
  // serial port, any received data passes to ParseByteData() function.
  void Send();

  // Function is running as a separate thread of process and it is always reading
  // data from TUN /dev/net/tun, then encodes data to HDLC format frame, sends data
  // through serial port by using FrameEncodeToHdlcAndSend() function.
  void Receive();

  // The main function which runs Listen() and Distribute() functions as a separate
  // threads or asynchronous.
  void Run();
};

#endif

