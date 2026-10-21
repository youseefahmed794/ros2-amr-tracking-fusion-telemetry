# License, references, and provenance

The existing MIT license, copyright 2026 Youseef Ahmed, and the MIT declaration in `package.xml` are retained without alteration.

The original project documentation states that initial Python/C++ architecture and code were supplied in project briefs. It does not identify their author or redistribution terms. That provenance needs confirmation before publication; the project license alone cannot resolve rights in externally supplied material. No separate vendored library or additional third-party copyright notice was found in the selected source files. This inspection cannot establish provenance of unattributed material.

Technical references remain credited:

- [Kalibr IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model), for the gyro random-walk model.
- [NASA consider-state discussion](https://ntrs.nasa.gov/api/citations/20120010515/downloads/20120010515.pdf), for partial state updates and retained uncertainty.
- Gazebo Harmonic [DiffDrive](https://gazebosim.org/docs/harmonic/moving_robot/), [sensor](https://gazebosim.org/docs/harmonic/sensors/), and [ROS integration](https://gazebosim.org/docs/harmonic/ros2_integration/) documentation, for plugin/bridge configuration.
- [rclpy timing discussion](https://github.com/ros2/rclpy/issues/1223), as context for scheduling observations, not proof of their cause.

ROS 2, Gazebo, NumPy, Matplotlib, PyYAML, pytest, WebSockets, and other externally installed dependencies retain their own licenses. The project license does not replace their notices. Redistributed container packages also retain their individual license requirements.
