#!/bin/bash
# Run after sourcing the built workspace. Publishes motion commands: no robots.
set -euo pipefail
: "${ROS_DOMAIN_ID:?Set an isolated ROS_DOMAIN_ID without a physical robot}"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${1:-/tmp/amr-runtime-validation}"
mkdir -p "$output_dir"
for controller in cpp python; do
  python3 "$project_dir/scripts/hybrid_smoke_test.py" --controller "$controller" \
    > "$output_dir/hybrid_${controller}.log" 2>&1
  for scenario in nominal imu_dropout odom_dropout odom_delay nonfinite command_dropout; do
    python3 "$project_dir/scripts/simulation_smoke_test.py" --controller "$controller" \
      --scenario "$scenario" --output "$output_dir/${controller}_${scenario}.json" \
      > "$output_dir/${controller}_${scenario}.log" 2>&1
  done
done
