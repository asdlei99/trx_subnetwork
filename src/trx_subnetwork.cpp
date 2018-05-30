#include "trx_subnetwork.h"

#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_tun.h>

// Function send a byte data through serial port.
//
// Arguments:
// - data: A data which will be sent.
void TrxSubNetwork::SendByte(const uint8_t data) {
  // TODO Note that currently I keep temporary variable in order to convert and
  // pass to write function for now this is OK. As soon as we change serial port
  // to VHF we should modify current function.
  const char byte = (char)data;
  serial_stream.write(&byte, 1);

  // Wait until the data has actually been transmitted.
  serial_stream.DrainWriteBuffer();
}



// Function finds valid HDLC frame from incoming data.
// If frame data received correct including frame flags (byte by byte at a time)
// it will store frame data to received_frame_buffer_ array and calls
// HandleFrameData() function, which processes frame.
//
// Arguments:
// - data: Received byte data from serial port.
void TrxSubNetwork::ParseByteData(uint8_t data) {

  HdlcState next_state = HdlcState::START;

  switch (current_state_) {
    case HdlcState::START:
         if (data == FRAME_BOUNDARY_FLAG)
           next_state = HdlcState::FRAME;
         else
           next_state = HdlcState::START;
    break;

    case HdlcState::FRAME:
         if (data != FRAME_BOUNDARY_FLAG && data != CONTROL_ESCAPE_OCTET) {
           if (frame_position_ <= MAX_FRAME_LENGTH) {
             received_frame_buffer_[frame_position_] = data;
             frame_position_++;
             next_state = HdlcState::FRAME;
           } else {
             frame_position_ = 0;
             next_state = HdlcState::START;
           }
         } else if (data == FRAME_BOUNDARY_FLAG) {
           if (frame_position_ <= MAX_FRAME_LENGTH && frame_position_ > 0) {
             HandleFrameData(received_frame_buffer_, frame_position_);
           }
           frame_position_ = 0;
           next_state = HdlcState::START;
         } else if (data == CONTROL_ESCAPE_OCTET) {
           next_state = HdlcState::ESCAPE;
         }
    break;

    case HdlcState::ESCAPE:
         data ^= INVERT_OCTET;
         if (data == CONTROL_ESCAPE_OCTET || data == FRAME_BOUNDARY_FLAG) {
           if (frame_position_ <= MAX_FRAME_LENGTH) {
             received_frame_buffer_[frame_position_] = data;
             frame_position_++;
             next_state = HdlcState::FRAME;
           } else {
             frame_position_ = 0;
             next_state = HdlcState::START;
           }
         } else {
           frame_position_ = 0;
           next_state = HdlcState::START;
         }
    break;

    default:
      current_state_ = HdlcState::START;
  }

  current_state_ = next_state;
}



// Function gets as an input frame, data encodes it to HDLC frame format and
// sends it out byte at a time using SendByte() function.
//
// Arguments:
// - frame_buffer: Frame data or original frame.
// - frame_length: Frame length in bytes.
void TrxSubNetwork::FrameEncodeToHdlcAndSend(const uint8_t* frame_buffer,
                                             uint32_t frame_length) {

  uint8_t data = 0;

  SendByte((uint8_t)FRAME_BOUNDARY_FLAG);

  while (frame_length) {
    data = *frame_buffer;
    frame_buffer++;
    if (data == CONTROL_ESCAPE_OCTET || data == FRAME_BOUNDARY_FLAG) {
      SendByte((uint8_t)CONTROL_ESCAPE_OCTET);
      data ^= (uint8_t)INVERT_OCTET;
    }

    SendByte(data);
    frame_length--;
  }

  SendByte((uint8_t)FRAME_BOUNDARY_FLAG);
}



// Function processes received frame, basically writes data to to TUN /dev/net/tun
//
// Arguments:
// - frame_data: Received frame data.
// - frame_length: A length of frame.
void TrxSubNetwork::HandleFrameData(const uint8_t* frame_data,
                                    const uint32_t frame_length) {

  if (frame_data != nullptr && frame_length > 0) {
    ssize_t bytes_write = write(tun_fd_, frame_data, frame_length);
    if (bytes_write < 0) {
      std::cerr << "An error occurred in the TUN write.\n";
    }
  }
}



// Network device ("/dev/net/tun") allocation.
//
// Arguments:
// - dev: It should be the name of the device with a format string. (e.g. "tun%d").
//        Note that the character pointer becomes overwritten with the real device
//        name (e.g. "tun0").
//
// Return the file descriptor of the new tun device.
int TrxSubNetwork::TunAlloc(char* dev) {
  if (dev == NULL)
    return -1;

  int fd = open("/dev/net/tun", O_RDWR);
  if (fd < 0)
    return fd;

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));

  ifr.ifr_flags = IFF_TUN | IFF_NO_PI;
  std::strncpy(ifr.ifr_name, dev, IFNAMSIZ);

  int code = ioctl(fd, TUNSETIFF, (void*)&ifr);
  if (code < 0)
    return code;

  std::strncpy(dev, ifr.ifr_name, IFNAMSIZ);

  return fd;
}



// Function is running as a separate thread of process and it is always listening
// serial port, any received data passes to ParseByteData() function.
void TrxSubNetwork::Listen() {
  // Variable to store data coming from the serial port.
  char data_byte;

  while (1) {
    try {
      // Keep reading data from serial port.
      while (serial_stream.IsDataAvailable()) {
        // Read a single byte of data from the serial port.
        serial_stream.get(data_byte);
        ParseByteData((uint8_t)data_byte);
      }
    } catch (const std::exception &e) {
      std::cout << "Network listening error: " << e.what() << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}



// Function is running as a separate thread of process and it is always reading
// data from TUN /dev/net/tun, then encodes data to HDLC format frame, sends data
// through serial port by using FrameEncodeToHdlcAndSend() function.
void TrxSubNetwork::Distribute() {

  uint8_t* buffer = new uint8_t[MAX_FRAME_LENGTH];
  std::memset(buffer, 0, MAX_FRAME_LENGTH);

  ssize_t bytes_read = -1;
  while (true) {
    try {
      bytes_read = read(tun_fd_, buffer, sizeof(buffer));
      if (bytes_read >= 0) {
        FrameEncodeToHdlcAndSend(buffer, bytes_read);
      } else {
        std::cerr << "An error occurred in the TUN read.\n";
      }
    } catch (const std::exception &e) {
      std::cout << "Data sending error: " << e.what() << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 4));
  }
}



// The main function which runs Listen() and Distribute() functions as a separate
// threads or asynchronous.
void TrxSubNetwork::Run() {

  std::thread listen_thread(&Listen, this);
  std::thread distribute_thread(&Distribute, this);

  listen_thread.join();
  distribute_thread.join();
}

