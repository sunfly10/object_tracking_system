from launch import LaunchDescription #런치 전체 구성을 만드는 핵심 클래스
from launch_ros.actions import Node #ROS 2 노드를 실행하는 액션

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flask_camera_serial',
            executable='angle_calculation_transmission',
            name='angle_calc_trans',
            output='screen',
        ),

        Node(
            package='flask_camera_serial', # 노드가 들어 있는 패키지 이름
            executable='flask_camera', # setup.py에 등록한 콘솔 스크립트 이름
            name='rpi_cam_pub', # ROS 노드 이름
            output='screen', # 터미널에 출력 보여줌
        ),
        Node(
            package='flask_camera_serial',
            executable='name_number_transmission',
            name='name_number_trans',
            output='screen',
        )
    ])
