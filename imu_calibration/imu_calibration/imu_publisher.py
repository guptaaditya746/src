import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2 as smbus
import math
import time
import numpy as np
import traceback


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.5, self.publish_imu_data)  # 2 Hz
        self.bus = smbus.SMBus(1)
        self.mpu_addr = 0x68  # Default I2C address for MPU6050

        # Wake up the MPU6050
        self.bus.write_byte_data(self.mpu_addr, 0x6B, 0)

        # Calibration parameters
        self.calibrated = False
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.accel_scale = 1.0

        self.samples_required = 500
        self.gyro_data = []
        self.accel_data = []

        self.get_logger().info("IMU Publisher initialized. Starting calibration...")
        self.calibrate_imu()

    def read_word(self, reg):
        try:
            high = self.bus.read_byte_data(self.mpu_addr, reg)
            low = self.bus.read_byte_data(self.mpu_addr, reg + 1)
            value = (high << 8) + low
            if value >= 0x8000:  # Handle negative values
                value = -((65535 - value) + 1)
            return value
        except IOError as e:
            self.get_logger().warn(f"I2C Read Error: {e}")
            return 0  # Return a default value in case of error

    def read_scaled(self, reg, scale):
        return self.read_word(reg) / scale

    def calibrate_imu(self):
        self.get_logger().info("Calibrating IMU... Please keep the sensor stationary.")

        for _ in range(self.samples_required):
            # Read raw accelerometer and gyroscope data
            accel_x = self.read_scaled(0x3B, 16384.0)
            accel_y = self.read_scaled(0x3D, 16384.0)
            accel_z = self.read_scaled(0x3F, 16384.0)
            gyro_x = self.read_scaled(0x43, 131.0)
            gyro_y = self.read_scaled(0x45, 131.0)
            gyro_z = self.read_scaled(0x47, 131.0)

            # Collect data for bias calculation
            self.gyro_data.append([gyro_x, gyro_y, gyro_z])
            self.accel_data.append([accel_x, accel_y, accel_z])

            time.sleep(0.01)  # Small delay between readings

        # Convert to numpy arrays
        self.gyro_data = np.array(self.gyro_data)
        self.accel_data = np.array(self.accel_data)

        # Calculate biases
        self.gyro_bias = np.mean(self.gyro_data, axis=0)
        self.accel_bias = np.mean(self.accel_data, axis=0)

        # Optional: Scale accelerometer to match gravity
        accel_magnitude = np.linalg.norm(self.accel_bias)
        self.accel_scale = 9.81 / accel_magnitude

        self.calibrated = True
        self.get_logger().info("IMU Calibration complete.")
        self.get_logger().info(f"Gyro Bias: {self.gyro_bias}")
        self.get_logger().info(f"Accel Bias: {self.accel_bias}, Scale: {self.accel_scale}")

    def publish_imu_data(self):
        imu_msg = Imu()

        # Read raw accelerometer and gyroscope data
        accel_x = self.read_scaled(0x3B, 16384.0)
        accel_y = self.read_scaled(0x3D, 16384.0)
        accel_z = self.read_scaled(0x3F, 16384.0)
        gyro_x = self.read_scaled(0x43, 131.0)
        gyro_y = self.read_scaled(0x45, 131.0)
        gyro_z = self.read_scaled(0x47, 131.0)

        if self.calibrated:
            # Apply calibration
            accel_x = (accel_x - self.accel_bias[0]) * self.accel_scale
            accel_y = (accel_y - self.accel_bias[1]) * self.accel_scale
            accel_z = (accel_z - self.accel_bias[2]) * self.accel_scale
            gyro_x -= self.gyro_bias[0]
            gyro_y -= self.gyro_bias[1]
            gyro_z -= self.gyro_bias[2]

        # Populate IMU message
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance[0] = -1

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                               0.0, 0.01, 0.0,
                                               0.0, 0.0, 0.01]

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                  0.0, 0.1, 0.0,
                                                  0.0, 0.0, 0.1]

        # Publish the IMU message
        self.publisher_.publish(imu_msg)
        self.get_logger().info("Published calibrated IMU data")


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
