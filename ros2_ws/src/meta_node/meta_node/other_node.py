import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt6.QtCore import QTimer

from std_msgs.msg import Int32, Float32

class WSReceiver(Node):
    door = "Closed"
    drawer_top = "Closed"
    drawer_bottom = "Closed"
    def __init__(self, label_door, label_top_drawer, label_bottom_drawer):
        super().__init__('wm_receiver')
        self.label_door = label_door
        self.label_top_drawer = label_top_drawer
        self.label_bottom_drawer = label_bottom_drawer

        self.subscription_door = self.create_subscription(
            Int32,
            'washing_machine/door',
            self.listener_callback_door,
            10)
        self.subscription_top_drawer = self.create_subscription(
            Float32,
            'washing_machine/top_drawer',
            self.listener_callback_top_drawer,
            10)
        self.subscription_bottom_drawer = self.create_subscription(
            Float32,
            'washing_machine/bottom_drawer',
            self.listener_callback_bottom_drawer,
            10)

    def listener_callback_door(self, msg):
        self.get_logger().info('I receive door information: "%d"' % msg.data)
        self.door = "Closed" if msg.data else "Opened"
        self.label_door.setText(f'Door: {self.door}')

    def listener_callback_top_drawer(self, msg):
        flag = False
        self.get_logger().info('I receive top drawer information: "%.1f"' % msg.data)
        if(msg.data == 0.):
            self.drawer_top = "Fully closed"
        elif(msg.data == 1.):
            self.drawer_top = "Fully opened"
        else:
            flag = True
        if(flag):
            self.label_top_drawer.setText(f'Top Drawer: {msg.data * 100:.1f}% opened')
        else:
            self.label_top_drawer.setText(f'Top Drawer: {self.drawer_top}')

    def listener_callback_bottom_drawer(self, msg):
        flag = False
        self.get_logger().info('I receive bottom drawer information: "%.1f"' % msg.data)
        if(msg.data == 0.):
            self.drawer_top = "Fully closed"
        elif(msg.data == 1.):
            self.drawer_top = "Fully opened"
        else:
            flag = True
        if(flag):
            self.label_top_drawer.setText(f'Top Drawer: {msg.data * 100:.1f}% opened')
        else:
            self.label_top_drawer.setText(f'Top Drawer: {self.drawer_top}')


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
    app = QApplication(sys.argv)

    main_window = MainWindow()
    main_window.show()

    node = WSReceiver(
        main_window.label_door,
        main_window.label_top_drawer,
        main_window.label_bottom_drawer)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(100)

    app.exec()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()