#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor

# --- Robot motor pins ---
motor_left = Motor(forward=3, backward=4, enable=18)
motor_right = Motor(forward=16, backward=19, enable=12)
MAX_SPEED = 0.5  # Max motor speed

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.get_logger().info("cmd_vel subscriber initialized.")

    def listener_callback(self, msg):
        # Compute wheel speeds from linear and angular velocities
        right_speed = (msg.linear.x - msg.angular.z) / 2.0
        left_speed = (msg.linear.x + msg.angular.z) / 2.0

        # Clamp speeds
        right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
        left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)

        # Apply speeds to motors
        if right_speed > 0:
            motor_right.forward(right_speed)
        elif right_speed < 0:
            motor_right.backward(abs(right_speed))
        else:
            motor_right.stop()

        if left_speed > 0:
            motor_left.forward(left_speed)
        elif left_speed < 0:
            motor_left.backward(abs(left_speed))
        else:
            motor_left.stop()


def stop_motors():
    motor_left.stop()
    motor_right.stop()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop motors safely
        stop_motors()
    finally:
        # Stop motors and destroy node
        stop_motors()
        node.destroy_node()
    # NO rclpy.shutdown() here

if __name__ == '__main__':
    main()

