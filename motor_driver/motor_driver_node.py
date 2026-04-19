import rclpy
from rclpy.node import Node
from motor_interfaces.msg import PWM
import lgpio

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            PWM,
            '/motor_pwm',
            self.callback,
            10
        )
        self.h = lgpio.gpiochip_open(4)
        self.left_pin = 19
        self.right_pin = 18
        lgpio.gpio_claim_output(self.h, self.left_pin)
        lgpio.gpio_claim_output(self.h, self.right_pin)
        self.get_logger().info("Motor driver initialized with lgpio")

    def set_pwm(self, pin, value):
        pulse = int(1500 + value * 250)
        lgpio.tx_servo(self.h, pin, pulse)

    def callback(self, msg):
        self.set_pwm(self.left_pin, msg.left_pwm)
        self.set_pwm(self.right_pin, msg.right_pwm)
        self.get_logger().info(
            f"PWM L={msg.left_pwm:.2f} R={msg.right_pwm:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
