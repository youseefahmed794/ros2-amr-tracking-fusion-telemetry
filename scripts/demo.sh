#!/bin/bash
# Build this standalone package and launch the numerical dynamics simulation.
set -e
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
colcon --log-base "$project_dir/log" build --base-paths "$project_dir" \
  --build-base "$project_dir/build" --install-base "$project_dir/install" \
  --packages-select ros2_amr_nmpc_telemetry --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$project_dir/install/setup.bash"
exec ros2 launch ros2_amr_nmpc_telemetry amr_simulation.launch.py "$@"
