import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta
from std_msgs.msg import Bool
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QPushButton
from PyQt6.QtCore import QTimer, Qt

class MetaNode(Node):
    def __init__(self, gui):
        super().__init__('timer_points')
        self.gui = gui
        self.start_datetime = None
        self.timer_running = False

        self.declare_parameters(
            namespace='', 
            parameters=[
                ('capteur1', rclpy.Parameter.Type.STRING), 
                ('capteur2', rclpy.Parameter.Type.STRING)
            ]
        )
        capteur1 = self.get_parameter('capteur1').get_parameter_value().string_value
        capteur2 = self.get_parameter('capteur2').get_parameter_value().string_value
        self.start_subscriber = self.create_subscription(
            Bool,
            capteur1,
            self.start_timer,
            10)
        self.end_subscriber = self.create_subscription(
            Bool,
            capteur2,
            self.end_timer,
            10)
        
        self.chrono_timer = QTimer()
        self.chrono_timer.timeout.connect(self.update_chronometer)

    def start_timer(self, msg):
        if not self.timer_running and msg.data:
            self.start_datetime = datetime.now()
            self.get_logger().info("Start")
            self.gui.update_status("Started")
            self.gui.update_time("00m:00s:000ms")
            self.timer_running = True
            self.chrono_timer.start(10)  # Update every 10ms

    def end_timer(self, msg):
        if self.timer_running and msg.data:
            self.timer_running = False
            self.chrono_timer.stop()
            end_datetime = datetime.now()
            self.get_logger().info("End")
            difference_time = end_datetime - self.start_datetime
            time_str = self.format_time(difference_time)
            self.gui.update_status("Ended")
            self.gui.update_time(time_str)
            self.get_logger().info(f"Time: {time_str}")

    def reset_timer(self):
        self.timer_running = False
        self.chrono_timer.stop()
        self.start_datetime = None
        self.gui.update_status("Waiting")
        self.gui.update_time("00m:00s:000ms")
        self.get_logger().info("Timer reset")

    def update_chronometer(self):
        if self.start_datetime:
            current_time = datetime.now()
            elapsed_time = current_time - self.start_datetime
            time_str = self.format_time(elapsed_time)
            self.gui.update_time(time_str)

    @staticmethod
    def format_time(time_delta):
        minutes, seconds = divmod(time_delta.total_seconds(), 60)
        milliseconds = time_delta.microseconds // 1000
        return f"{int(minutes)}m:{int(seconds)}s:{int(milliseconds)}ms"

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Race")
        self.layout = QVBoxLayout()

        self.title_label = QLabel("Robot Race")
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("""
            font-size: 36px; 
            font-weight: bold; 
            color: #333; 
            background-color: #f0f0f0; 
            padding: 20px;
        """)
        self.layout.addWidget(self.title_label)

        self.status_label = QLabel("Status: Waiting")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("""
            font-size: 18px; 
            color: #555;
            padding: 10px;
        """)
        self.layout.addWidget(self.status_label)

        self.time_label = QLabel("Time: 00m:00s:000ms")
        self.time_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.time_label.setStyleSheet("""
            font-size: 24px; 
            color: #007ACC;
            font-weight: bold;
            padding: 20px;
        """)
        self.layout.addWidget(self.time_label)

        self.reset_button = QPushButton("Restart")
        self.reset_button.setStyleSheet("""
            font-size: 18px; 
            padding: 10px;
        """)
        self.layout.addWidget(self.reset_button)

        self.setLayout(self.layout)
        self.resize(600, 400)

    def update_status(self, status):
        self.status_label.setText(f"Status: {status}")

    def update_time(self, time_str):
        self.time_label.setText(f"Time: {time_str}")

    def set_reset_button_callback(self, callback):
        self.reset_button.clicked.connect(callback)

def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])
    gui = MainWindow()
    gui.show()

    node = MetaNode(gui)

    gui.set_reset_button_callback(node.reset_timer)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(100)  # 100 ms

    app.exec()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
