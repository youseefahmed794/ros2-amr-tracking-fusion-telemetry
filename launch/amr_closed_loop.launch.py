from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def selected_controller(context):
    controller = LaunchConfiguration('controller').perform(context)
    if controller not in ('cpp', 'python'):
        raise ValueError('controller must be cpp or python')
    requested_rate = float(LaunchConfiguration('control_rate_hz').perform(context))
    if requested_rate < 0:
        raise ValueError('control_rate_hz must be nonnegative (0 selects controller default)')
    common = {'timing_output': LaunchConfiguration('timing_output').perform(context), 'kx': 1.4, 'ky': 2.8, 'ktheta': 2.0, 'ki_v': 0.3,
              'v_max': 1.5, 'omega_max': 2.0, 'state_timeout_sec': 0.5}
    if controller == 'cpp':
        return [
            Node(package='ros2_amr_nmpc_telemetry', executable='adaptive_controller_cpp_node',
                 output='screen', parameters=[common, {'control_rate_hz': requested_rate or 100.0}]),
            Node(package='ros2_amr_nmpc_telemetry', executable='telemetry_bridge_node',
                 output='screen'),
        ]
    return [Node(package='ros2_amr_nmpc_telemetry', executable='adaptive_controller_node',
                 output='screen', parameters=[common, {'control_rate_hz': requested_rate or 50.0,
                                                     'executor_threads': int(LaunchConfiguration('executor_threads').perform(context))}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('timing_output', default_value=''),
        DeclareLaunchArgument('executor_threads', default_value='1'),
        DeclareLaunchArgument('controller', default_value='cpp', choices=['cpp', 'python'],
                              description='Select exactly one cmd_vel publisher'),
        Node(package='ros2_amr_nmpc_telemetry', executable='ekf_localization_node',
             output='screen', parameters=[{'publish_rate_hz': 50.0,
                                           'process_noise_pos': 0.01,
                                           'meas_noise_odom': 0.04}]),
        OpaqueFunction(function=selected_controller),
    ])
