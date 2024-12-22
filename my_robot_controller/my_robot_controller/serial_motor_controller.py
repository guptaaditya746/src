import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SerialMotorController(Node):
    def __init__(self):
        super().__init__('serial_motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Subscribe to the /cmd_vel topic
            self.cmd_vel_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Update with your Arduino port

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x  # Extract the linear velocity from the Twist message
        if linear_x > 0:
            self.serial_port.write(b"FORWARD\n")
            self.get_logger().info("Command sent: FORWARD")
        elif linear_x < 0:
            self.serial_port.write(b"BACKWARD\n")
            self.get_logger().info("Command sent: BACKWARD")
        else:
            self.serial_port.write(b"STOP\n")
            self.get_logger().info("Command sent: STOP")

def main(args=None):
    rclpy.init(args=args)
    motor_controller = SerialMotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

