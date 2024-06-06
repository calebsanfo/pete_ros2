import time
import board
import adafruit_mpu6050
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.mpu = adafruit_mpu6050.MPU6050(self.i2c)

        # Create a timer that will call the timer_callback function every second
        self.timer = self.create_timer(.005, self.timer_callback)

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu"

        # Read acceleration data
        acceleration = self.mpu.acceleration
        msg.linear_acceleration.x = acceleration[0]
        msg.linear_acceleration.y = acceleration[1]
        msg.linear_acceleration.z = acceleration[2]

        # Read gyroscope data
        gyro = self.mpu.gyro
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        # MPU6050 does not provide orientation (quaternion)
        msg.orientation = Quaternion()

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published IMU data: {msg}")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)

    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
