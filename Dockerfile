FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]
ENV OPENBLAS_NUM_THREADS=1
WORKDIR /opt/amr_ws
COPY . src/ros2_amr_nmpc_telemetry
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-rosdep python3-matplotlib \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /var/lib/apt/lists/*
ENTRYPOINT ["/opt/amr_ws/src/ros2_amr_nmpc_telemetry/scripts/docker_entrypoint.sh"]
CMD ["ros2", "launch", "ros2_amr_nmpc_telemetry", "amr_simulation.launch.py"]
