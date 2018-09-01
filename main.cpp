#include <dronecore/dronecore.h>
#include <dronecore/telemetry.h>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace std::this_thread;

#define ERROR_CONSOLE_TEXT "\033[31m"      // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m"  // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m"      // Restore normal console colour

namespace {
volatile std::sig_atomic_t gSignalStatus;
}

void usage(std::string bin_name) {
  std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name
            << " <connection_url>" << std::endl
            << "Connection URL format should be :" << std::endl
            << " For TCP : tcp://[server_host][:server_port]" << std::endl
            << " For UDP : udp://[bind_host][:bind_port]" << std::endl
            << " For Serial : serial:///path/to/serial/dev[:baudrate]"
            << std::endl
            << "For example, to connect to the simulator use URL: udp://:14540"
            << std::endl;
}

void signal_handler(int signal) { gSignalStatus = signal; }

int main(int argc, char **argv) {
  dronecore::DroneCore dc;
  std::string connection_url;
  dronecore::ConnectionResult connection_result;

  bool discovered_system = false;
  if (argc == 2) {
    connection_url = argv[1];
    connection_result = dc.add_any_connection(connection_url);
  } else {
    usage(argv[0]);
    return 1;
  }

  if (connection_result != dronecore::ConnectionResult::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT << "Connection failed: "
              << connection_result_str(connection_result) << NORMAL_CONSOLE_TEXT
              << std::endl;
    return 1;
  }

  std::cout << "Waiting to discover system..." << std::endl;
  dc.register_on_discover([&discovered_system](uint64_t uuid) {
    std::cout << "Discovered system with UUID: " << uuid << std::endl;
    discovered_system = true;
  });

  // We usually receive heartbeats at 1Hz, therefore we should find a system
  // after around 2 seconds.
  sleep_for(seconds(2));

  if (!discovered_system) {
    std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting."
              << NORMAL_CONSOLE_TEXT << std::endl;
    return 1;
  }

  dronecore::System &system = dc.system();

  auto telemetry = std::make_shared<dronecore::Telemetry>(system);

  telemetry->attitude_euler_angle_async(
      [](dronecore::Telemetry::EulerAngle angles) {
        std::cout << "Roll: " << angles.roll_deg << ", "
                  << "Pitch: " << angles.pitch_deg << ", "
                  << "Yaw: " << angles.yaw_deg << std::endl;
      });

  while (gSignalStatus != SIGINT) {
    sleep_for(seconds(1));
  }

  return 0;
}
