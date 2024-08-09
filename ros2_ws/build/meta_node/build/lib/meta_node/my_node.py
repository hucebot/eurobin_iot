import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32

from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt6.QtCore import QTimer

class MetaNode(Node):

    def __init__(self, gui):
        super().__init__('washing_machine')
        self.gui = gui
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
            Int16,
            capteur2,
            self.listener_callback_top_drawer,
            10)
        self.subscription_bottom_drawer = self.create_subscription(
            Int16,
            capteur3,
            self.listener_callback_bottom_drawer,
            10)
        self.subscription_door
        self.subscription_top_drawer
        self.subscription_bottom_drawer

    def listener_callback_door(self, msg):
        self.get_logger().info('I heard door information: "%d"' % msg.data)
        door = "Closed" if msg.data else "Opened"
        self.gui.label_door.setText(f'Door: {door}')

    def listener_callback_top_drawer(self, msg):
        flag = False
        self.get_logger().info('I heard distance top drawer: "%d"' % msg.data)
        new_msg = Float32()
        if (msg.data <= 20):
            drawer_top = "Fully closed"
        elif (msg.data > 480):
            drawer_top = "Fully opened"
        else:
            data = msg.data / 480
            flag = True
        if(flag):
            self.gui.label_top_drawer.setText(f'Top Drawer: {data * 100:.1f}% opened')
        else:
            self.gui.label_top_drawer.setText(f'Top Drawer: {drawer_top}')

    def listener_callback_bottom_drawer(self, msg):
        flag = False
        self.get_logger().info('I heard distance bottom drawer: "%d"' % msg.data)
        if (msg.data <= 20):
            drawer_bottom = "Fully closed"
        elif (msg.data > 480):
            drawer_bottom = "Fully opened"
        else:
            data = msg.data / 480
            flag=True
        if(flag):
            self.gui.label_bottom_drawer.setText(f'Top Drawer: {data * 100:.1f}% opened')
        else:
            self.gui.label_bottom_drawer.setText(f'Top Drawer: {drawer_bottom}')

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.resize(300, 300)
        self.label_door = QLabel('Door: N/A')
        self.label_top_drawer = QLabel('Top Drawer: N/A')
        self.label_bottom_drawer = QLabel('Bottom Drawer: N/A')

        layout = QVBoxLayout()
        layout.addWidget(self.label_door)
        layout.addWidget(self.label_top_drawer)
        layout.addWidget(self.label_bottom_drawer)
        self.setLayout(layout)

        self.setWindowTitle('Washing Machine')


def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])

    gui = MainWindow()
    gui.show()

    node = MetaNode(gui)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(100)

    app.exec()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()