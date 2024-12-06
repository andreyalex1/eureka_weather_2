from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_weather_2',
            executable='cwt_uwd_reader',
            name='cwt_uwd_reader',
            shell=True,
        ),
    ])