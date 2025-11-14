from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # Caminhos
    pkg_share = FindPackageShare('slam_toolbox').find('slam_toolbox')
    config_dir = os.path.join(os.getenv('HOME'), 'ros2_ws', 'src', 'config')
    config_file = os.path.join(config_dir, 'slam_config.yaml')
    sdf_path = os.path.join(os.getenv('HOME'), 'ros2_ws', 'main.sdf')

    # Executar Gazebo com seu mundo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', sdf_path],
        output='screen'
    )

    # Bridge do LIDAR
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # Publicador de estado do robô
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[os.path.join(os.getenv('HOME'), 'ros2_ws', 'main.sdf')]
    )


    # SLAM Toolbox (modo mapeamento)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
    gazebo,
    lidar_bridge,
    robot_state_publisher,
    slam_toolbox
    ])

