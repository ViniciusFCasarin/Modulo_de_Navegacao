from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """
    Launch file para controle manual do robô via teclado.
    
    Uso:
    Terminal 1: ros2 launch robot_bringup gazebo_world.launch.py
    Terminal 2: ros2 launch robot_bringup mapping.launch.py
    Terminal 3: ros2 launch robot_bringup teleop_keyboard.launch.py
    
    Controles:
    - Use as teclas i/j/k/l/u/o/m/,/. para controlar o robô
    - q/z para aumentar/diminuir velocidade
    - Espaço para parar
    """
    
    # Argumento de tempo simulado
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Bridge ROS <-> Gazebo para /cmd_vel (necessário se não estiver rodando em outro launch)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',  # ROS2 → Gazebo (unidirecional)
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Nó de teleoperação via teclado com velocidades REDUZIDAS
    # Executa com parâmetros inline para velocidades baixas:
    # - linear: 0.18 m/s (equivalente a 0.2 com 1x diminuição)
    # - angular: 0.27 rad/s (equivalente a 0.3 com 1x diminuição)
    # Use q/z no teclado para aumentar/diminuir conforme necessário
    teleop_node = ExecuteProcess(
        cmd=[
            'xterm', '-e',
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', 
            '-p', 'speed:=0.18',
            '-p', 'turn:=0.27'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        bridge_node,
        teleop_node
    ])
