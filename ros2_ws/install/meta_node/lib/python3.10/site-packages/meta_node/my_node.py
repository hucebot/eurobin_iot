import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32


class MetaNode(Node):

    def __init__(self):
        super().__init__('washing_machine')

        self.declare_parameters(
            namespace='', 
            parameters=[
                ('capteur1', rclpy.Parameter.Type.STRING), 
                ('capteur2', rclpy.Parameter.Type.STRING), 
                ('capteur3', rclpy.Parameter.Type.STRING)
            ]
        )
        capteur1 = self.get_parameter('capteur1').get_parameter_value().string_value
        capteur2 = self.get_parameter('capteur2').get_parameter_value().string_value
        capteur3 = self.get_parameter('capteur3').get_parameter_value().string_value
        self.subscription_door = self.create_subscription(
            Int32,
            capteur1,
            self.listener_callback_door,
            10)
        self.subscription_top_drawer = self.create_subscription(
            Int16MultiArray,
            capteur2,
            self.listener_callback_top_drawer,
            10)
        self.subscription_bottom_drawer = self.create_subscription(
            Int16MultiArray,
            capteur3,
            self.listener_callback_bottom_drawer,
            10)
        self.publisher_door = self.create_publisher(Int32, 'washing_machine/door', 10)
        self.publisher_top_drawer = self.create_publisher(Float32, 'washing_machine/top_drawer', 10)
        self.publisher_bottom_drawer = self.create_publisher(Float32, 'washing_machine/bottom_drawer', 10)
        self.subscription_door
        self.subscription_top_drawer
        self.publisher_bottom_drawer

    def listener_callback_door(self, msg):
        self.get_logger().info('I heard door information: "%d"' % msg.data)
        new_msg = Int32()
        new_msg.data = msg.data
        self.publisher_door.publish(new_msg)

    def listener_callback_top_drawer(self, msg):
        self.get_logger().info('I heard distance top drawer: "%d"' % msg.data[0])
        new_msg = Float32()
        if (msg.data[0] <= 20):
            new_msg.data = 0.
        elif (msg.data[0] > 480):
            new_msg.data = 1.
        else:
            new_msg.data = msg.data[0] / 480
        self.publisher_top_drawer.publish(new_msg)

    def listener_callback_bottom_drawer(self, msg):
        self.get_logger().info('I heard distance bottom drawer: "%d"' % msg.data[0])
        new_msg = Float32()
        if (msg.data[0] <= 20):
            new_msg.data = 0.
        elif (msg.data[0] > 480):
            new_msg.data = 1.
        else:
            new_msg.data = msg.data[0] / 480
        self.publisher_bottom_drawer.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MetaNode()

    rclpy.spin(minimal_subscriber)
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()