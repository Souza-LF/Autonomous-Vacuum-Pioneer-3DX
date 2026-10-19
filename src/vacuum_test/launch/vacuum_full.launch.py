#!/usr/bin/env python3
import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    home_dir = os.path.expanduser('~')
    ws_dir = join(home_dir, 'Desktop', 'Projects', 'pioneer_ws')

    # Caminhos dos pacotes e arquivos de configuração
    p3dx_pkg = get_package_share_directory('p3dx_description')
    slam_pkg = get_package_share_directory('slam_toolbox')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    slam_params = join(ws_dir, 'src', 'Pioneer-3DX-ROS2', 'config', 'pioneer_mapper_params.yaml')
    nav2_params = join(ws_dir, 'src', 'Pioneer-3DX-ROS2', 'config', 'pioneer_nav2_params.yaml')
    rviz_config = join(ws_dir, 'vacuum.rviz')

    # 1. Simulação Gazebo (Inicia imediatamente - t = 0s)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(p3dx_pkg, 'launch', 'gz.launch.py'))
    )

    # 2. SLAM Toolbox (Inicia em t = 3s, quando o Gazebo já está publicando /clock e /scan)
    slam_toolbox = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(slam_pkg, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': slam_params
                }.items()
            )
        ]
    )

    # 3. Nav2 (Inicia em t = 5s, quando o /map do SLAM já existe)
    nav2_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(nav2_pkg, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params
                }.items()
            )
        ]
    )

    # 4. RViz2 (Inicia em t = 5s)
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # 5. Planejador Boustrophedon (Inicia em t = 7s e fica aguardando /mapping_done)
    boustrophedon_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='vacuum_test',
                executable='boustrophedon_planner',
                name='boustrophedon_planner',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # 6. Wall Follower (Inicia em t = 8s para começar a mapear a sala)
    wall_follower_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='vacuum_test',
                executable='wall_follower',
                name='wall_follower_node',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gz_sim,
        slam_toolbox,
        nav2_bringup,
        rviz_node,
        boustrophedon_node,
        wall_follower_node,
    ])
