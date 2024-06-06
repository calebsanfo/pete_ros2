import time
import serial
import adafruit_gps
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self.gps = adafruit_gps.GPS(self.uart, debug=False)

        # Set NMEA output and update rate to 10 Hz
        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK220,100")

        # Create a timer that will call the timer_callback function every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_print = time.monotonic()

    def timer_callback(self):
        self.gps.update()
        if not self.gps.has_fix:
            self.get_logger().info("Waiting for fix...")
            return
        
        # We have a fix! Create a NavSatFix message and populate it with GPS data
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        
        msg.status.status = NavSatStatus.STATUS_FIX if self.gps.has_fix else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        msg.latitude = self.gps.latitude
        msg.longitude = self.gps.longitude
        msg.altitude = self.gps.altitude_m if self.gps.altitude_m is not None else float('nan')
        
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        msg.position_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published GPS fix: {msg}")

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)

    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
