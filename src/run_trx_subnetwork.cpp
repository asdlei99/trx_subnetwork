#include "trx_subnetwork.h"

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

int main() {

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

  TrxSubNetwork trx_subnetwork;
  trx_subnetwork.Run();

  return 0;
}

