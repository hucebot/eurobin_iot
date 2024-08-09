import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from PyQt6.QtWidgets import QApplication, QMainWindow, QListWidget, QVBoxLayout, QWidget
from PyQt6.QtCore import QTimer




class MetaNode(Node):

    products = {
        "e28069950000400acd13529c": "product1",
        "e28069950000500acd134e9c": "product2"
    }

    id_non_match = {}
    id_match = {}
    id = []

    def __init__(self, list):
        super().__init__('fridge')
        self.list = list
        self.declare_parameters(
            namespace='', 
            parameters=[
                ('capteur1', rclpy.Parameter.Type.STRING)
            ]
        )
        capteur1 = self.get_parameter('capteur1').get_parameter_value().string_value
        self.subscription_door = self.create_subscription(
            String,
            capteur1,
            self.listener_callback_rfid,
            10)
        self.subscription_door

    def listener_callback_rfid(self, msg):
        self.get_logger().info('I heard ID information: "%s"' % msg.data)
        self.show_id(msg)
        

    def show_id(self, msg):
        if msg.data == "Nothing":
            for key in self.id:
                self.id_non_match[key] += 1
                if self.id_non_match[key] >= 3:
                    del self.id_non_match[key]
                    del self.id_match[key]
                    self.id.remove(key)
                    
        elif msg.data in self.id:
            self.id_non_match[msg.data] = 0
            for key in self.id:
                if key != msg.data:
                    self.id_non_match[key] += 1
                    if self.id_non_match[key] >= 3:
                        del self.id_non_match[key]
                        del self.id_match[key]
                        self.id.remove(key)
        else:
            if msg.data in self.id_match.keys():
                self.id_match[msg.data] += 1
                if self.id_match[msg.data] >= 1:
                    self.id.append(msg.data)
            else:
                self.id_match[msg.data] = 0
            self.id_non_match[msg.data] = 0
        self.list.clear()
        self.list.addItems([self.products[x] for x in self.id if x in self.products.keys()])


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ID Display")
        self.setGeometry(100, 100, 400, 300)

        self.list_widget = QListWidget()

        layout = QVBoxLayout()
        layout.addWidget(self.list_widget)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])
    main_window = MainWindow()

    main_window.show()

    node = MetaNode(main_window.list_widget)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(100)

    
    app.exec()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()