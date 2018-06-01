#include "trx_subnetwork.h"

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

int main(int argc, char* argv[]) {

  // Initialize default values.
  int baud_rate = 115200;
  std::string input_serial_dev = "ttyS0";
  std::string input_tun_dev = "tun0";

  if (argc != 4 && argc != 2) {
    std::cout << "Usage 1: " << argv[0] << " <baud rate> <serial port device> <TUN device name>\n";
    std::cout << "Usage 2: " << argv[0] << " <baud rate>\n";
    std::cout << "WARNING: Default values are set.\n";
    std::cout << "Serial port speed: 115200\n"
              << "Serial port device: ttyS0\n"
              << "TUN device: tun0\n";
  } else if (argc == 2) {
      baud_rate = std::atoi(argv[1]);
      std::cout << "Serial port and TUN devices are set by default.\n";
      std::cout << "Serial port device: ttyS0\n"
                << "TUN device: tun0\n";
  } else {
    baud_rate = std::atoi(argv[1]);
    input_serial_dev = std::string(argv[2]);
    input_tun_dev = std::string(argv[3]);
  }

  // TODO Verify/check and see if input values are provided correct.

  /* TODO Uncomment once project will be released in order to run it as a daemon.
  // Process ID. Create child process.
  const pid_t pid = fork();
  if (pid < 0) {
    std::cerr << "fork failed!\n";
    exit(EXIT_FAILURE);
  }

  // If PID > 0, then we can exit the parent process, because child can continue
  // to run even after the parent has finished executing
  if (pid > 0) {
    exit(EXIT_SUCCESS);
  }

  // Create a new SID (Session ID) for the child process.
  const pid_t sid = setsid();
  if (sid < 0) {
    exit(EXIT_FAILURE);
  }
  */

  TrxSubNetwork trx_subnetwork(baud_rate, input_serial_dev, input_tun_dev);
  trx_subnetwork.Run();

  return 0;
}

