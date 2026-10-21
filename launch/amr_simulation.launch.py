"""One-command headless or RViz closed-loop differential-drive simulation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('ros2_amr_nmpc_telemetry'))
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('scenario', default_value='nominal', choices=[
            'nominal','load','slip','imu_dropout','odom_dropout','odom_delay','nonfinite','command_dropout']),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('controller', default_value='cpp', choices=['cpp','python']),
        Node(package='ros2_amr_nmpc_telemetry', executable='simulator_node', output='screen',
             parameters=[str(share/'config/simulation.yaml'),
                         {'scenario': LaunchConfiguration('scenario'),
                          'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int)}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (share/'urdf/amr.urdf').read_text()}]),
        Node(package='ros2_amr_nmpc_telemetry', executable='visualization_node'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amr_closed_loop.launch.py')),
                                 launch_arguments={'controller': LaunchConfiguration('controller')}.items()),
        Node(package='rviz2', executable='rviz2', arguments=['-d',str(share/'rviz/amr.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
