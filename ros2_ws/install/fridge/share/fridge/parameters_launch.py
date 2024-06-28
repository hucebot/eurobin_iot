from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fridge',
            executable='listener',
            name='fridge',
            parameters=[
                {'capteur1': 'eurobin_iot_63/rfid'}  #eurobin_iot_ID/name of the sensor
            ]
        )
    ])
