import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Definição do caminho do arquivo de mundo
    world_path = PathJoinSubstitution([
        FindPackageShare('ros2_gazeboworld'),
        'world',
        'meu_mundo.sdf'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use sim time'
        ),
        DeclareLaunchArgument(
            'world', default_value=world_path, description='Path to the world file'
        ),

        # Iniciar o Ignition Gazebo com um mundo específico
        ExecuteProcess(
            cmd=['ign', 'gazebo', LaunchConfiguration('world')],
            output='screen'
        ),

        # Inicia as pontes entre o gazebo e o ros2
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/camera@sensor_msgs/msg/Image@ignition.msgs.Image'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/model/meu_carrin/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
            output='screen'
        ),
        
        
        
        
        # Iniciar o script ros2_bridge
        Node(
            package='ros2_gazeboworld',
            executable='ros2_bridge',
            name='ros2_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/camera', '/camera')]
        ),
    ])