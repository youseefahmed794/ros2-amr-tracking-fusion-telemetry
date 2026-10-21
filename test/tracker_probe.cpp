#include <iomanip>
#include <iostream>
#include "ros2_amr_nmpc_telemetry/tracker.hpp"

int main()
{
  amr_control::Tracker tracker;
  double x, y, yaw, t, dt;
  std::cout << std::setprecision(17);
  while (std::cin >> x >> y >> yaw >> t >> dt) {
    const auto cmd = tracker.step({x, y, yaw}, t, dt);
    std::cout << cmd[0] << " " << cmd[1] << "\n";
  }
}
