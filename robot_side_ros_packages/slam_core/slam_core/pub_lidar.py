
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster
import serial
import math
from . import lidar  # your lidar helper module
import numpy as np

# --- Constants ---
LIDAR_PORT = '/dev/ttyAMA0'       # Primary UART on Raspberry Pi Zero 2W
LIDAR_BAUDRATE = 115200
START_BIT = 0xFA
BUFFER_SIZE = 22
SAMPLES_PER_PACKET = 4
TOTAL_ANGLES = 360

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        # --- ROS publisher ---
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.001, self.poll_serial)
        self.get_logger().info('laser_scan publisher initialized.')

        # --- Serial connection ---
        self.serial = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1)

        # --- State variables ---
        self.last_scan_time = 0.0
        self.packet = bytearray(BUFFER_SIZE)
        self.buffer_index = 0
        self.raw_data_old = 0
        self.raw_data_curr = 0

        self.angles:np.ndarray = np.zeros(shape=90)
        self.ranges:np.ndarray = np.zeros(shape=360)
        self.intensities:np.ndarray = np.zeros(shape=360)
        self.now = self.get_clock().now()

        self.msg = LaserScan()
        self.msg.angle_min = 0.0
        self.msg.angle_max = 2.0 * math.pi
        self.msg.range_min = 0.20
        self.msg.range_max = 6.0
        self.msg.angle_increment = math.pi / 180.0
        self.msg.header.frame_id = 'laser_frame'  # Match the TF child frame
    # --------------------------------------------------------------
    def poll_serial(self):

        self.packet = bytearray(BUFFER_SIZE)
        self.buffer_index = 0
        self.raw_data_old = 0
        self.raw_data_curr = 0

        while True:
            temp = self.serial.read(1)
            if not temp:
                continue
            temp = temp[0]
            self.raw_data_old = self.raw_data_curr
            self.raw_data_curr = temp

            if self.raw_data_old == START_BIT and 0xA0 <= self.raw_data_curr <= 0xF9:

                if 0xA0 == self.raw_data_curr:

                    self.now = self.get_clock().now()
                    now_sec = self.now.nanoseconds / 1e9
                    self.msg.header.stamp = self.now.to_msg()

                    self.msg.scan_time = now_sec - self.last_scan_time
                    self.msg.time_increment = self.msg.scan_time / 360.0
                    self.msg.ranges = self.ranges
                    self.msg.intensities = self.intensities

                    self.publisher_.publish(self.msg)
                    self.last_scan_time = now_sec

                    self.packet[0] = START_BIT
                    self.packet[1] = self.raw_data_curr
                    self.buffer_index = 2
                    continue
                
                if lidar.verify_packet_checksum(self.packet): # if packet correcly read

                    self.process_packet(self.packet)

                    self.packet[0] = START_BIT
                    self.packet[1] = self.raw_data_curr
                    self.buffer_index = 2
                    continue

            if self.buffer_index >= BUFFER_SIZE:
                self.buffer_index = 0
                continue

            self.packet[self.buffer_index] = self.raw_data_curr
            self.buffer_index += 1

    # --------------------------------------------------------------
    def process_packet(self, packet:bytearray):
        '''Insert measurements into the arrays'''

        self.angles[lidar.angle(packet)] = float(lidar.angle(packet))

        self.ranges[4*lidar.angle(packet)    ] = lidar.dist_m(packet[4:8])
        self.ranges[4*lidar.angle(packet) + 1] = lidar.dist_m(packet[8:12])
        self.ranges[4*lidar.angle(packet) + 2] = lidar.dist_m(packet[12:16])
        self.ranges[4*lidar.angle(packet) + 3] = lidar.dist_m(packet[16:20])

        self.intensities[4*lidar.angle(packet)    ] = lidar.signal_strength(packet[6:10])
        self.intensities[4*lidar.angle(packet) + 1] = lidar.signal_strength(packet[10:14])
        self.intensities[4*lidar.angle(packet) + 2] = lidar.signal_strength(packet[14:18])
        self.intensities[4*lidar.angle(packet) + 3] = lidar.signal_strength(packet[18:22])


# --------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    rclpy.spin(node)
    node.serial.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
