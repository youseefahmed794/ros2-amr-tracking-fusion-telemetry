cmake_minimum_required(VERSION 3.16)
project(ros2_amr_nmpc_telemetry VERSION 1.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(adaptive_controller_cpp_node src/adaptive_controller_node.cpp)
target_include_directories(adaptive_controller_cpp_node PRIVATE include)
ament_target_dependencies(adaptive_controller_cpp_node rclcpp geometry_msgs nav_msgs std_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(adaptive_controller_cpp_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
install(TARGETS adaptive_controller_cpp_node DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})
foreach(node ekf_localization_node adaptive_controller_node telemetry_bridge_node simulator_node visualization_node command_watchdog_node)
  # Wrappers use absolute package imports, so ros2 run works after installation.
  install(PROGRAMS scripts/${node}.py DESTINATION lib/${PROJECT_NAME} RENAME ${node})
  install(PROGRAMS scripts/${node}.py DESTINATION lib/${PROJECT_NAME})
endforeach()
install(PROGRAMS scripts/standalone_benchmark_colab.py scripts/ros_smoke_test.py scripts/hybrid_smoke_test.py scripts/run_experiments.py scripts/simulation_smoke_test.py scripts/measure_timing.py scripts/validate_gyro_bias.py scripts/long_localization_check.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch docs config urdf worlds rviz DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE THIRD_PARTY_NOTICES.md README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(python_core test/test_core.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV OPENBLAS_NUM_THREADS=1)
  ament_add_pytest_test(gyro_bias test/test_bias.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV OPENBLAS_NUM_THREADS=1)
  ament_add_pytest_test(simulation_faults test/test_simulation.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV OPENBLAS_NUM_THREADS=1)
  ament_add_pytest_test(simulation_assets test/test_assets.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(runtime_nodes test/test_runtime_nodes.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=183 ROS_LOG_DIR=/tmp/amr-runtime-node-tests OPENBLAS_NUM_THREADS=1
    TIMEOUT 30)
  add_executable(tracker_probe test/tracker_probe.cpp)
  target_include_directories(tracker_probe PRIVATE include)
  ament_add_pytest_test(cpp_python_parity test/test_cpp_parity.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV TRACKER_PROBE=$<TARGET_FILE:tracker_probe> OPENBLAS_NUM_THREADS=1)
  add_executable(cpp_ros_smoke test/cpp_ros_smoke.cpp)
  target_include_directories(cpp_ros_smoke PRIVATE include)
  ament_target_dependencies(cpp_ros_smoke rclcpp geometry_msgs nav_msgs std_msgs)
  add_test(NAME cpp_ros_smoke COMMAND cpp_ros_smoke)
  set_tests_properties(cpp_ros_smoke PROPERTIES TIMEOUT 30 ENVIRONMENT "ROS_LOG_DIR=/tmp/amr-cpp-test-logs;ROS_DOMAIN_ID=175")
endif()
ament_package()
