import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor
import threading
import time

# Define motors (GPIO pins)
motor2 = Motor(forward=3, backward=4, enable=18)    # left  motor
motor1 = Motor(forward=16, backward=19, enable=12)  # right motor

# Smoothly move 'current' toward 'target' by 'step' increments.
def approach(current, target, step=0.05):
    if abs(current - target) < step:
        return target
    return current + step if target > current else current - step

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback,10)

        # Shared state (protected by a lock)
        # self.lock = threading.Lock()
        self.right_wheel_speed = 0.0
        self.left_wheel_speed = 0.0
        self.right_wheel_speed_stable = 0.0
        self.left_wheel_speed_stable = 0.0

        self.get_logger().info("cmd_vel listener initialized with threaded control loops.")

        # Start background threads for each motor
        # self.running = True
        # self.m_thread = threading.Thread(target=self.wheel_loop, daemon=True)
        # self.m_thread.start()

    # Update target speeds when new cmd_vel message arrives.
    def listener_callback(self, msg):
        # with self.lock:
        #     self.left_wheel_speed = (msg.linear.x + msg.angular.z) / 2
        #     self.right_wheel_speed = (msg.linear.x - msg.angular.z) / 2

        #     # Clamp speeds to safe range [-0.5, 0.5]
        #     self.left_wheel_speed = max(min(self.left_wheel_speed, 0.5), -0.5)
        #     self.right_wheel_speed = max(min(self.right_wheel_speed, 0.5), -0.5)

        self.right_wheel_speed = ((msg.linear.x - msg.angular.z) / 2.0)
        self.left_wheel_speed = ((msg.linear.x + msg.angular.z) / 2.0)

        # Clamp speeds to safe range [-0.5, 0.5]
        self.right_wheel_speed = max(min(self.right_wheel_speed, 0.5), -0.5)
        self.left_wheel_speed = max(min(self.left_wheel_speed, 0.5), -0.5)

        if self.right_wheel_speed > 0.0:
            motor1.forward(self.right_wheel_speed)
        elif self.right_wheel_speed  < 0.0:
            motor1.backward(abs(self.right_wheel_speed))
        else:
            motor1.stop()

        if self.left_wheel_speed > 0.0:
            motor2.forward(self.left_wheel_speed)
        elif self.left_wheel_speed < 0.0:
            motor2.backward(abs(self.left_wheel_speed))
        else:
            motor2.stop()

        

    def wheel_loop(self):
        while self.running:

            with self.lock:
                target_right = self.right_wheel_speed
                current_right = self.right_wheel_speed_stable
                target_left = self.left_wheel_speed
                current_left = self.left_wheel_speed_stable

            new_speed_right = approach(current_right, target_right)
            new_speed_left = approach(current_left, target_left)

            if new_speed_right > 0:
                motor1.forward(new_speed_right)
            elif new_speed_right < 0:
                motor1.backward(abs(new_speed_right))
            else:
                motor1.stop()

            if new_speed_left > 0:
                motor2.forward(new_speed_left)
            elif new_speed_left < 0:
                motor2.backward(abs(new_speed_left))
            else:
                motor2.stop()

            with self.lock:
                self.right_wheel_speed_stable = new_speed_right   
            
            with self.lock:
                self.left_wheel_speed_stable = new_speed_left   
                
            time.sleep(0.001)  # ~100Hz update rate

    # Stop threads before shutting down node.
    def destroy_node(self):
        # self.running = False
        # time.sleep(0.1)
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
