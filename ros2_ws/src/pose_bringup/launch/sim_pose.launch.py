# ~/ros2_ws/src/pose_bringup/launch/sim_pose.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare # Para encontrar o caminho do pacote


def generate_launch_description():
    
    # --- 1. CONFIGURAÇÃO DE CAMINHOS ---
    
    # Localize o arquivo de mundo que contém o plugin de pose.
    # *** SUBSTITUA 'main.sdf' pelo nome do seu arquivo SDF ***
    world_name = 'main.sdf' 

    # Assumindo que seu arquivo de mundo está no pacote 'pose_bringup' (se não estiver, substitua o nome do pacote)
    world_path = PathJoinSubstitution([
        FindPackageShare('pose_bringup'), 
        'worlds', # Ou o subdiretório onde seu SDF está
        world_name
    ])

    # Localize o launch file padrão do ros_gz_sim (que inicia o Gazebo)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    
    # --- 2. AÇÕES DE LANÇAMENTO ---

    # A. Iniciar o Gazebo Server e GUI
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments={'gz_args': [' -r -s ', world_path]}.items(), # -r e -s garantem que o simulador comece rodando
    )
    
    # B. NÓ DA PONTE (ROS <-> GZ)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Configuração de pose (OUTPUT Gazebo -> ROS 2)
            # Tópico Gazebo: /pose/robot/current (do plugin SDF)
            # Tipo ROS 2: geometry_msgs/msg/PoseStamped
            # Tipo Gazebo: gz.msgs.Pose (Mensagem de pose de um único objeto)
            '/pose/robot/current@geometry_msgs/msg/PoseStamped@gz.msgs.Pose', # Note a sintaxe com colchetes
            
            # Você pode adicionar o cmd_vel aqui para garantir que a ponte o veja:
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        remappings=[
            # 1. REMAPEAMENTO DA POSE (para ROS 2)
            ('/pose/robot/current', '/robot_pose'),
            
            # 2. (OPCIONAL) Remapeamento para cmd_vel, se seu robô precisar disso
            # ('/cmd_vel', '/model/NOME_DO_SEU_ROBÔ/cmd_vel') # Substitua pelo tópico do seu diff_drive
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        bridge_node,
    ])
