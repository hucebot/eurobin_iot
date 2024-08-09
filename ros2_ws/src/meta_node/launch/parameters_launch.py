from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='meta_node',
            executable='listener',
            name='washing_machine',
            parameters=[
                {'capteur1': 'eurobin_iot_0/hall'},  #eurobin_iot_ID/name of the sensor
                {'capteur2': 'eurobin_iot_38/tofm4'},
                {'capteur3': 'eurobin_iot_95/tofm4'}
            ]
        )
    ])
