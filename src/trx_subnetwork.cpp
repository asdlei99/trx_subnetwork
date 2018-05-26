#include "trx_subnetwork.h"

#include <iostream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>


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
  // FRAME FLAG
  if (data == FRAME_BOUNDARY_FLAG) {
    if (escape_character_ == true) {
      escape_character_ = false;
    } else { // If a valid frame is detected.
      const int msb = (received_frame_buffer_[frame_position_ - 1] << 8);
      const int lsb = (received_frame_buffer_[frame_position_ - 2] & 0xff);
      // (msb << 8 ) | (lsb & 0xff)
      // TODO I am not sure, but I think I have a bug here as this never happens.
      // It may be because of crc_ccitt_update() function.
      if (frame_position_ >= 2 && frame_checksum_ == (msb | (lsb))) {
        HandleFrameData(received_frame_buffer_, frame_position_ - 2);
      }

      frame_position_ = 0;
      frame_checksum_ = CRC16_CCITT_INIT_VAL;
      return;
    }
  }

  if (escape_character_) {
    escape_character_ = false;
    data ^= INVERT_OCTET;
  } else if (data == CONTROL_ESCAPE_OCTET) {
    escape_character_ = true;
    return;
  }

  received_frame_buffer_[frame_position_] = data;

  if (frame_position_ - 2 >= 0) {
    frame_checksum_ = crc_ccitt_update(frame_checksum_, received_frame_buffer_[frame_position_ - 2]);
  }

  frame_position_++;

  if (frame_position_ == frame_length_) {
    HandleFrameData(received_frame_buffer_, frame_length_);

    frame_position_ = 0;
    frame_checksum_ = CRC16_CCITT_INIT_VAL;
  }
}



// The following is the equivalent functionality written in C.
uint16_t TrxSubNetwork::crc_ccitt_update(const uint16_t crc, uint8_t data) {

  data ^= (crc & 255);
  data ^= data << 4;

  const uint16_t value = ((((uint16_t)data << 8) | (crc >> 8)) ^
                          (uint8_t)(data >> 4)  ^
                          ((uint16_t)data << 3));
  return value;
}



// Function gets as an input frame, data encodes it to HDLC frame format and
// sends it out byte at a time using SendByte() function.
//
// Arguments:
// - frame_buffer: Frame data or original frame.
// - frame_length: Frame length in bytes.
void TrxSubNetwork::FrameEncodeToHdlcAndSend(const uint8_t* frame_buffer,
                                             uint8_t frame_length) {

  // Just double check and make sure that function gets correct data.
  if (frame_buffer == nullptr || frame_length != FRAME_LENGTH) {
    std::cout << "Not correct data is provided to encode to HDLC frame format.\n";
    return;
  }

  uint8_t data = 0;
  uint16_t fcs = CRC16_CCITT_INIT_VAL;

  SendByte((uint8_t)FRAME_BOUNDARY_FLAG);

  while (frame_length) {
    data = *frame_buffer;
    frame_buffer++;
    fcs = crc_ccitt_update(fcs, data);
    if ((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_FLAG)) {
      SendByte((uint8_t)CONTROL_ESCAPE_OCTET);
      data ^= (uint8_t)INVERT_OCTET;
    }

    SendByte(data);
    frame_length--;
  }

  data = low(fcs);
  if ((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_FLAG)) {
    SendByte((uint8_t)CONTROL_ESCAPE_OCTET);
    data ^= (uint8_t)INVERT_OCTET;
  }
  SendByte(data);

  data = high(fcs);
  if ((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_FLAG)) {
    SendByte((uint8_t)CONTROL_ESCAPE_OCTET);
    data ^= (uint8_t)INVERT_OCTET;
  }

  SendByte(data);
  SendByte((uint8_t)FRAME_BOUNDARY_FLAG);
}



// Function processes received frame, basically writes data to to TUN /dev/net/tun
//
// Arguments:
// - frame_data: Received frame data.
// - frame_length: A length of frame.
void TrxSubNetwork::HandleFrameData(const uint8_t* frame_data,
                                    const uint16_t frame_length) {

  if (frame_data != nullptr && frame_length > 0) {
    // TODO write data to TUN /dev/net/tun
    char byte = 0;
    std::cout << "Received data: ";
    for (int i = 0; i < frame_length; i++) {
      byte = (char)frame_data[i];
      std::cout << byte;
    }
  }
  std::cout << "\n";
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

  // TODO For now we just read data from file and send it, later I should
  // read data from /dev/net/tun
  std::ifstream input_file("file.txt");
  // Determine if the input file argument is valid to read data from.
  if (!input_file.good()) {
    std::cerr << "Error: Could not open file for reading.\n";
    return;
  }

  uint8_t* data = new uint8_t[FRAME_LENGTH];
  std::memset(data, 0, FRAME_LENGTH);

  unsigned int idx = 0;
  char byte = 0;
  while (true) {
    try {
      idx = 0;
      byte = 0;
      while (input_file >> byte && idx < FRAME_LENGTH) {
        data[idx++] = (uint8_t)byte;
      }

      FrameEncodeToHdlcAndSend(data, FRAME_LENGTH);

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

