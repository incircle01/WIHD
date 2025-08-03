import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SDF 모델 경로
    pkg_desc = get_package_share_directory('morphing_description')
    sdf_path = os.path.join(pkg_desc, 'model', 'Morphing_drone.sdf')

    return LaunchDescription([
        # 1) Gazebo + ROS2 플러그인
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_init.so',
                sdf_path
            ],
            output='screen'
        ),

        # TimerAction(
        #     period=3.45,  # Gazebo가 완전히 올라온 뒤 1초 뒤에 Reset
        #     actions=[
        #         ExecuteProcess(
        #             cmd=[
        #                 'ros2', 'service', 'call', '/reset_world', 'std_srvs/srv/Empty', '{}'
        #             ],
        #             output='screen'
        #         )
        #     ]
        # ),

        # 2) 컨트롤러 노드 (5초 지연 실행)
        TimerAction(
            period=10.0,  # 3.5초 후 실행
            actions=[
                Node(
                    package='morphing_drone_control',
                    executable='main_controller',
                    name='morphing_drone_controller',
                    output='screen',
                    # parameters=[os.path.join(pkg_desc, 'config', 'your_params.yaml')],
                )
            ]
        ),

        # 3) 모터 퍼블리셔 노드
        Node(
            package='motor_speeds_pub',
            executable='motor_speeds_pub',
            name='motor_speeds_pub',
            output='screen',
        ),
    ])
