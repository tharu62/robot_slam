#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from gpiozero import RotaryEncoder

# Robot constants
RADIUS = 0.06           # wheel radius [m]
WHEELBASE = 0.3         # distance between wheels [m]
MAX_STEPS_LEFT = 1410
MAX_STEPS_RIGHT = 1410


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Publisher and TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Encoders
        self.encoder_left = RotaryEncoder(22, 23)
        self.encoder_right = RotaryEncoder(6, 5)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timing
        self.last_time = self.get_clock().now()

        # Publish odometry at 10 Hz
        self.create_timer(0.1, self.publish_odometry)

        self.get_logger().info("OdomPublisher node started.")

    def publish_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # Read encoder steps (right inverted if necessary)
        steps_left = float(self.encoder_left.steps)
        steps_right = float(-self.encoder_right.steps)

        # Reset for relative step count
        self.encoder_left.steps = 0
        self.encoder_right.steps = 0

        # Convert to distances
        D_left = steps_left * 2 * math.pi * RADIUS / MAX_STEPS_LEFT
        D_right = steps_right * 2 * math.pi * RADIUS / MAX_STEPS_RIGHT
        D_avg = (D_left + D_right) / 2.0
        delta_theta = (D_right - D_left) / WHEELBASE

        # Update orientation (normalize)
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update position
        delta_x = D_avg * math.cos(self.theta)
        delta_y = D_avg * math.sin(self.theta)
        self.x += delta_x
        self.y += delta_y

        # Velocities
        v = D_avg / dt
        w = delta_theta / dt

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
