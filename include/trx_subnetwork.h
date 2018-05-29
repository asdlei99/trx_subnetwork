#ifndef TRX_SUB_NETWORK_H
#define TRX_SUB_NETWORK_H

#include <stdint.h>

#include "irt_serial_port.h"
#include <SerialStream.h>
using namespace LibSerial;

// The frame boundary flag '~' (7E in hexadecimal) marks as a beginning
// and end of frame.
#define FRAME_BOUNDARY_FLAG 0x7E

// A "control escape octet" flag '}' (7D in hexadecimal).
#define CONTROL_ESCAPE_OCTET 0x7D

// If either of these two octets appears in the transmitted data, an escape octet is sent,
// followed by the original data octet with bit 5 inverted.
#define INVERT_OCTET 0x20

// 16 bit low and high bytes copier.
#define low(x)  ((x) & 0xFF)
#define high(x) (((x)>>8) & 0xFF)

// Length of frame in bytes.
#define FRAME_LENGTH 4


struct TrxSubNetwork {

  bool escape_character_;
  uint16_t frame_position_;
  uint16_t frame_length_;
  uint8_t* received_frame_buffer_;

  // Note that currently we use serial port communication between two TRX
  // machines, later is has to be VHF based communication.
  SerialStream serial_stream;


  // Constructor.
  //
  // Arguments:
  // - frame_length: Frame length in bytes, note that in both side it has to
  //                 be the same. By default it is set value of FRAME_LENGTH.
  //
  // - baud_rate: Serial port speed (baud rate). By default it is set 115200.
  TrxSubNetwork(const uint16_t frame_length = FRAME_LENGTH,
                const int baud_rate = 115200) :
      escape_character_(false),
      frame_position_(0),
      frame_length_(frame_length),
      received_frame_buffer_(new uint8_t[frame_length_ + 1]) {

        // Instantiate a SerialStream object.
        SetupSerialPort(serial_stream, baud_rate);
    };


  // Destructor.
  ~TrxSubNetwork() {
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
  void SendByte(const uint8_t data);

  // Function gets as an input frame data, encodes it to HDLC frame format and
  // sends it out byte at a time using SendByte() function.
  //
  // Arguments:
  // - frame_buffer: Frame data or original frame.
  // - frame_length: Frame length in bytes.
  void FrameEncodeToHdlcAndSend(const uint8_t* frame_buffer, uint8_t frame_length);

  // Function processes received frame, basically writes data to to TUN /dev/net/tun
  //
  // Arguments:
  // - frame_data: Received frame data.
  // - frame_length: A length of frame.
  void HandleFrameData(const uint8_t* frame_data, const uint16_t frame_length);

  // Network device allocation.
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
  void Listen();

  // Function is running as a separate thread of process and it is always reading
  // data from TUN /dev/net/tun, then encodes data to HDLC format frame, sends data
  // through serial port by using FrameEncodeToHdlcAndSend() function.
  void Distribute();

  // The main function which runs Listen() and Distribute() functions as a separate
  // threads or asynchronous.
  void Run();
};

#endif

