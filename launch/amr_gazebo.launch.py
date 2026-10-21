"""Optional Gazebo Harmonic integration; requires ros-jazzy-ros-gz."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    share = Path(get_package_share_directory('ros2_amr_nmpc_telemetry'))
    try:
        gz = Path(get_package_share_directory('ros_gz_sim'))
        get_package_share_directory('ros_gz_bridge')
    except PackageNotFoundError as exc:
        raise RuntimeError('Gazebo integration requires ros-jazzy-ros-gz; use amr_simulation.launch.py for the tested headless plant') from exc
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    args = f'-r {share / "worlds/warehouse.sdf"}' + ('' if gui else ' -s')
    sim = {'use_sim_time': True}
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz/'launch/gz_sim.launch.py')),
                                 launch_arguments={'gz_args': args}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': (share/'urdf/amr_gazebo.urdf').read_text()}]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world','amr_lab','-name','amr','-topic','/robot_description','-z','0.01']),
        Node(package='ros2_amr_nmpc_telemetry', executable='command_watchdog_node'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share/'config/gazebo_bridge.yaml')}]),
        Node(package='ros2_amr_nmpc_telemetry', executable='ekf_localization_node', parameters=[sim]),
        Node(package='ros2_amr_nmpc_telemetry', executable='adaptive_controller_cpp_node', parameters=[sim]),
        Node(package='ros2_amr_nmpc_telemetry', executable='telemetry_bridge_node', parameters=[sim]),
        Node(package='ros2_amr_nmpc_telemetry', executable='visualization_node', parameters=[sim]),
        Node(package='rviz2', executable='rviz2', parameters=[sim],
             arguments=['-d',str(share/'rviz/amr.rviz')], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('gui', default_value='true'),
                              DeclareLaunchArgument('rviz', default_value='true'),
                              OpaqueFunction(function=launch_nodes)])
