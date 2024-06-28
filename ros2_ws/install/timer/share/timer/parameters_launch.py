from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='timer',
            executable='listener',
            name='timer_points',
            parameters=[
                {'capteur1': 'eurobin_iot_95/time'},  #eurobin_iot_ID/name of the sensor
                {'capteur2': 'eurobin_iot_63/time'}
            ]
        )
    ])
