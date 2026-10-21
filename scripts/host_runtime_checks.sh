#!/bin/bash
# Execute in a real host terminal, after dependency installation. No physical robot.
set -e
if [[ ${1:-} == --help || $# != 2 ]]; then
  echo "Usage: $0 {websocket|gazebo|docker} ABSOLUTE_EVIDENCE_DIRECTORY"
  echo 'Runs only remaining checks; does not rerun comparative experiments.'
  [[ ${1:-} == --help ]]
  exit $?
fi
mode="$1"
evidence="$2"
[[ "$evidence" == /* ]] || { echo 'Evidence directory must be absolute' >&2; exit 2; }
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
source /opt/ros/jazzy/setup.bash
if [[ -f .venv/bin/activate ]]; then source .venv/bin/activate; fi
source install/setup.bash
set -uo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-186}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/amr-host-validation-logs}"
export OPENBLAS_NUM_THREADS=1
mkdir -p "$evidence"
case "$mode" in
  websocket)
    # Fail before ROS launch if this is accidentally invoked in the restricted agent.
    python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); s.close()'
    for controller in cpp python; do
      python3 scripts/hybrid_smoke_test.py --controller "$controller" \
        --output "$evidence/websocket_${controller}.json" 2>&1 | tee "$evidence/websocket_${controller}.log"
    done
    ;;
  gazebo)
    export GZ_PARTITION="${GZ_PARTITION:-amr_release_${USER}_$$}"
    python3 scripts/gazebo_runtime_check.py --output "$evidence/gazebo.json" \
      2>&1 | tee "$evidence/gazebo.log"
    ;;
  docker)
    sudo docker build --progress=plain -t amr-demo . 2>&1 | tee "$evidence/docker-build.log"
    sudo docker run --rm -e ROS_DOMAIN_ID=186 -v "$evidence:/evidence" amr-demo bash -c '
      set -e
      colcon test --packages-select ros2_amr_nmpc_telemetry --return-code-on-test-failure
      colcon test-result --verbose
      project=src/ros2_amr_nmpc_telemetry
      python3 "$project/scripts/hybrid_smoke_test.py" --controller cpp --output /evidence/docker_websocket_cpp.json
      python3 "$project/scripts/hybrid_smoke_test.py" --controller python --output /evidence/docker_websocket_python.json
      python3 "$project/scripts/simulation_smoke_test.py" --scenario nominal --output /evidence/docker_headless.json
    ' 2>&1 | tee "$evidence/docker-tests.log"
    ;;
  *) echo "Unknown mode: $mode" >&2; exit 2 ;;
esac
