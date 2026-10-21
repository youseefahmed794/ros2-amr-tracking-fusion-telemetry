# Local publication preparation — 2026-10-21

The candidate is `publication/ros2-amr-tracking-fusion-telemetry-20261021` inside the original project. The original source, existing changes, and earlier publication directory were preserved. No GitHub repository was created, no push was made, and no Git history was copied. No applicable `AGENTS.md` was found in the project or checked ancestor directories. The visible original `.git` directory is empty/read-only; `git status --short` and `git remote -v` both failed with “not a git repository.” Original tracked/untracked status, remotes, and history therefore could not be verified.

## Executed checks

| Check | Result |
|---|---|
| Separate C++17 Release build/install | Passed; one package built |
| Final complete colcon/CTest run | 37 reported results, zero errors/failures/skips |
| C++/Python parity | Passed, 3,000 stateful inputs |
| Offline experiment matrix | 120 trials completed; per-trial CSV and aggregate JSON exactly match the saved latest matrix |
| Numerical C++ nominal ROS smoke | Passed: truth/state/command samples, TF parents, path frames, wheel motion and publisher checks |
| Python/shell syntax, XML/YAML parsing | Passed: 43 Python, 4 shell, 8 XML, 2 URDF, 1 SDF, 2 YAML and 1 workflow YAML files |
| Saved artifact readability | Passed: 65 JSON, 14 CSV, 7 NPZ and 8 PNG files inspected before adding preparation artifacts |
| C++ WebSocket outside-sandbox retry | Passed: delivery, reconnect, sensor-loss zero payload, recovery and post-fault reconnect; observed command receipt 46.87 Hz |
| Installed numerical launch argument loading | Passed after setting a writable ROS log directory |
| Credential-pattern scan | No matches for the recorded private-key, GitHub-token, AWS-key, credentialed-URL and secret-assignment patterns in selected text files |
| License preservation | Original `LICENSE` is byte-identical |

The colcon count includes CTest wrappers and nested pytest results. It is the tool's reported count, not a claim of 37 independent experiments. [Current compact evidence](preparation-20261021/) is separate from historical `release/` evidence.

The first build attempt failed during installation because the rewritten README was not yet present; the completed candidate built successfully afterward. The first full suite timed out in `runtime_nodes` at its 30 s limit without producing XML. Both runtime tests then passed directly, the failed CTest group passed on retry, and a second complete colcon run finished with zero errors/failures/skips. The timeout cause is not established; DDS repeatedly emitted `getifaddrs: Operation not permitted` warnings. The first direct diagnostic accidentally replaced the ROS Python path and failed to import rclpy; preserving the sourced path resolved that diagnostic setup error. A launch-argument check initially tried the read-only default home log directory; setting `ROS_LOG_DIR` resolved it. Earlier failure logs are retained in the local preparation audit outside the candidate.

## Exact executed build and test commands

From the original project root `/home/youseef95/Documents/ros2_amr_nmpc_telemetry`:

```bash
source /opt/ros/jazzy/setup.bash
export OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/amr-publication-mpl
export ROS_DOMAIN_ID=193 ROS_LOG_DIR=/tmp/amr-publication-ros-logs
colcon --log-base /tmp/amr-publication-20261021/log build \
  --base-paths publication/ros2-amr-tracking-fusion-telemetry-20261021 \
  --build-base /tmp/amr-publication-20261021/build \
  --install-base /tmp/amr-publication-20261021/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /tmp/amr-publication-20261021/install/setup.bash
colcon --log-base /tmp/amr-publication-20261021/log test \
  --build-base /tmp/amr-publication-20261021/build \
  --install-base /tmp/amr-publication-20261021/install \
  --packages-select ros2_amr_nmpc_telemetry --return-code-on-test-failure
colcon test-result --test-result-base /tmp/amr-publication-20261021/build --verbose
# Targeted retry between the two full runs:
ctest --test-dir /tmp/amr-publication-20261021/build/ros2_amr_nmpc_telemetry \
  --rerun-failed --output-on-failure
```

From the candidate root with the installation sourced:

```bash
python3 scripts/run_experiments.py --output-dir /tmp/amr-publication-20261021/experiments
ROS_DOMAIN_ID=196 ROS_LOG_DIR=/tmp/amr-publication-simulation-logs \
  python3 scripts/simulation_smoke_test.py --controller cpp --scenario nominal \
  --output /tmp/amr-publication-20261021/ros-cpp-nominal.json
ROS_DOMAIN_ID=197 ROS_LOG_DIR=/tmp/amr-publication-websocket-logs \
  python3 scripts/hybrid_smoke_test.py --controller cpp \
  --output /tmp/amr-publication-20261021/websocket-cpp.json
# Same command outside the sandbox, with a new output path:
ROS_DOMAIN_ID=197 ROS_LOG_DIR=/tmp/amr-publication-websocket-logs \
  python3 scripts/hybrid_smoke_test.py --controller cpp \
  --output /tmp/amr-publication-20261021/websocket-cpp-retry.json
ROS_LOG_DIR=/tmp/amr-publication-launch-logs \
  ros2 launch ros2_amr_nmpc_telemetry amr_simulation.launch.py --show-args
```

Static checks used `ast.parse`, `bash -n`, `ElementTree.parse`, and `yaml.safe_load`. Artifacts were read with JSON/CSV parsers, `numpy.load(allow_pickle=False)` and Pillow image verification. The credential scan reports categories and file locations without echoing suspected values. These checks do not prove absence of every possible secret or establish authorship. Binary traces were checked for readable non-object arrays; no hidden-data/provenance guarantee is implied.

## Environment and verification limits

ROS Jazzy and Python 3.12.3 with preinstalled system dependencies and WebSockets 14.2 were available. CMake selected the original project's existing `.venv/bin/python3` for this local build. That environment was used without modification and is not copied. The candidate does not depend on its absolute path after a fresh configure; a clean-machine install is still unverified. Dependency versions appear in the README; GCC was queried afresh as 13.3.0 rather than assuming the historical package-meta version.

The WebSocket test initially failed with `PermissionError: Operation not permitted` opening its client socket. The outside-sandbox retry passed the C++ transport checks: delivery, disconnect/reconnect, sensor-loss stop, recovery payload, and post-fault reconnect. The recorded receipt rate was 46.87 Hz; this transport test does not establish the requested 100 Hz rate or worst-case deadlines. The Python WebSocket backend was not rerun. Docker is unavailable (`docker` executable absent), so image build/run remain unverified. Gazebo ROS integration packages are installed, but no new Gazebo or GUI session was run in this preparation. The saved Gazebo/RViz evidence is historical. The 110-trial long bias suite, multi-minute ROS runs, all twelve process fault/backend runs, and timing series were not repeated; their retained artifacts support historical claims only. GitHub Actions was inspected structurally but never dispatched. Clean-OS provisioning, Humble, hardware validation, hard real-time deadlines, slow-client stress and the complete Gazebo fault matrix remain unverified.

## Publication changes and decisions

The README and repository description describe integral nonlinear control accurately. The internal ROS name remains unchanged: renaming it affects package lookup, Python imports, C++ include paths, installation resources, launch/config/script references, CI/container commands, tests and evidence paths. A separate repository name avoids those compatibility changes.

Added a comprehensive `.gitignore`, a tightened Docker context ignore file, technical attribution/provenance notes, evidence scope notes and this preparation record. Added `python3-matplotlib` to the manifest because installed analysis helpers import it, and installed attribution beside the existing license. Runtime control/filter code and gains were not changed. Docker and Actions retain the same package name and build/test entry points as the source.

Generated build/install/log output, caches, virtual environments, private configuration, authentication/remotes diagnostics, redundant release artifacts and two large offline trace archives are excluded. No files were deleted from the original project. The local audit at `publication/preparation-20261021/` contains the exact selected/excluded inventory, original-file hashes, failed and successful check logs, and a final candidate checksum manifest. This audit directory is not part of the publication candidate.

The placeholder maintainer email `youseef@example.com` was replaced with `youseefahmed794@gmail.com` (Youseef Abdel Tawwab) in `package.xml` and the README. The existing MIT license was not selected or replaced during preparation. Redistribution rights for the initial project-brief material and required attribution should be confirmed by the maintainer. Original Git history must be inspected for sensitive content before any future decision to reuse it; this candidate contains none.
