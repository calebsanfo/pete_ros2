import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import adafruit_bno055

class BNO055Publisher(Node):

    def __init__(self):
        super().__init__('bno055_publisher')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(Imu, 'imu/bno055', 10)
        timer_period = .01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize the BNO055 sensor
        self.uart = serial.Serial(serial_port, baudrate=115200)  # Adjust baudrate as needed
        self.sensor = adafruit_bno055.BNO055_UART(self.uart)
        self.get_logger().info('BNO055 sensor initialized')

        # Switch to CONFIG_MODE to change settings
        # self.sensor.mode = adafruit_bno055.CONFIG_MODE

        # # Set higher update rates for accelerometer, gyroscope, and magnetometer
        # self.sensor.accel_bandwidth = adafruit_bno055.ACCEL_1000HZ
        # self.sensor.gyro_bandwidth = adafruit_bno055.GYRO_523HZ
        # self.sensor.magnet_rate = adafruit_bno055.MAGNET_30HZ

        # # Switch back to desired mode, e.g., NDOF_MODE
        # self.sensor.mode = adafruit_bno055.NDOF_MODE
        # self.get_logger().info('Sensor configuration updated and switched to NDOF mode')


    def timer_callback(self):
        imu_msg = Imu()
        
        # Populate the IMU message with data from the sensor
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Orientation (Quaternion)
        quat = self.sensor.quaternion
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]
        imu_msg.orientation.w = quat[0]

        # Angular velocity (Gyroscope)
        gyro = self.sensor.gyro
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        # Linear acceleration (Accelerometer)
        accel = self.sensor.linear_acceleration
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        # Publish the message
        self.publisher_.publish(imu_msg)
        #self.get_logger().info('Publishing IMU data')

def main(args=None):
    rclpy.init(args=args)
    bno055_publisher = BNO055Publisher()
    rclpy.spin(bno055_publisher)
    bno055_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
