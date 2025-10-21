import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor
import threading
import time

# Define motors (GPIO pins)
motor1 = Motor(forward=3, backward=4, enable=18)
motor2 = Motor(forward=16, backward=19, enable=12)

def approach(current, target, step=0.05):
    """Smoothly move 'current' toward 'target' by 'step' increments."""
    if abs(current - target) < step:
        return target
    return current + step if target > current else current - step

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

        # Shared state (protected by a lock)
        self.lock = threading.Lock()
        self.right_wheel_speed = 0.0
        self.left_wheel_speed = 0.0
        self.right_wheel_speed_stable = 0.0
        self.left_wheel_speed_stable = 0.0

        self.get_logger().info("cmd_vel listener initialized with threaded control loops.")

        # Start background threads for each motor
        self.running = True
        self.right_thread = threading.Thread(target=self.right_wheel_loop, daemon=True)
        self.left_thread = threading.Thread(target=self.left_wheel_loop, daemon=True)
        self.right_thread.start()
        self.left_thread.start()

    def listener_callback(self, msg):
        """Update target speeds when new cmd_vel message arrives."""
        with self.lock:
            self.right_wheel_speed = (msg.linear.x - msg.angular.z) / 2
            self.left_wheel_speed = (msg.linear.x + msg.angular.z) / 2

            # Clamp speeds to safe range [-0.5, 0.5]
            self.right_wheel_speed = max(min(self.right_wheel_speed, 0.5), -0.5)
            self.left_wheel_speed = max(min(self.left_wheel_speed, 0.5), -0.5)

    def right_wheel_loop(self):
        """Continuously adjust right wheel toward target speed."""
        while self.running:
            with self.lock:
                target = self.right_wheel_speed
                current = self.right_wheel_speed_stable

            # Gradually approach target
            new_speed = approach(current, target)

            # Apply motor command
            if new_speed > 0:
                motor1.forward(new_speed)
            elif new_speed < 0:
                motor1.backward(abs(new_speed))
            else:
                motor1.stop()

            with self.lock:
                self.right_wheel_speed_stable = new_speed

            time.sleep(0.05)  # ~20Hz update rate

    def left_wheel_loop(self):
        """Continuously adjust left wheel toward target speed."""
        while self.running:
            with self.lock:
                target = self.left_wheel_speed
                current = self.left_wheel_speed_stable

            # Gradually approach target
            new_speed = approach(current, target)

            # Apply motor command
            if new_speed > 0:
                motor2.forward(new_speed)
            elif new_speed < 0:
                motor2.backward(abs(new_speed))
            else:
                motor2.stop()

            with self.lock:
                self.left_wheel_speed_stable = new_speed

            time.sleep(0.05)  # ~20Hz update rate

    def destroy_node(self):
        """Stop threads before shutting down node."""
        self.running = False
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
