import rclpy
from rclpy.node import Node

from serial_interfaces.msg import SensorData
from motor_interfaces.msg import PWM


class HeadingController(Node):

    def __init__(self):
        super().__init__('heading_controller')

        # -----------------------------
        # SUB / PUB
        # -----------------------------
        self.subscription = self.create_subscription(
            SensorData,
            '/sensor_data',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            PWM,
            '/motor_pwm',
            10
        )

        # -----------------------------
        # CONTROL PARAMETERS
        # -----------------------------
        self.target_heading = 0.0

        self.kp = 0.6
        self.kd = 0.15

        # -----------------------------
        # FILTERING
        # -----------------------------
        self.alpha = 0.15
        self.yaw_filtered = 0.0

        # -----------------------------
        # STATE
        # -----------------------------
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        self.get_logger().info("Stable Heading Controller Started")

    # -----------------------------
    # UTIL: ANGLE WRAP
    # -----------------------------
    def wrap_angle(self, angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    # -----------------------------
    # MAIN CALLBACK
    # -----------------------------
    def callback(self, msg):

        # -------------------------
        # 1. TIME STEP
        # -------------------------
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if dt <= 0.0:
            dt = 0.05

        # -------------------------
        # 2. FILTER YAW
        # -------------------------
        raw_yaw = msg.yaw

        self.yaw_filtered = (
            self.alpha * raw_yaw +
            (1.0 - self.alpha) * self.yaw_filtered
        )

        # -------------------------
        # 3. ERROR (shortest path)
        # -------------------------
        error = self.wrap_angle(self.target_heading - self.yaw_filtered)

        # -------------------------
        # 4. DERIVATIVE (dt-aware)
        # -------------------------
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # -------------------------
        # 5. PD CONTROL
        # -------------------------
        control = self.kp * error + self.kd * derivative

        # -------------------------
        # 6. SATURATION
        # -------------------------
        control = max(-1.0, min(1.0, control))

        # -------------------------
        # 7. MOTOR MIXING
        # -------------------------
        base = 0.5

        left = base - control
        right = base + control

        left = max(0.0, min(1.0, left))
        right = max(0.0, min(1.0, right))

        # -------------------------
        # 8. PUBLISH PWM
        # -------------------------
        pwm_msg = PWM()
        pwm_msg.left_pwm = float(left)
        pwm_msg.right_pwm = float(right)

        self.publisher.publish(pwm_msg)

        # -------------------------
        # DEBUG
        # -------------------------
        self.get_logger().info(
            f"yaw={self.yaw_filtered:.2f} "
            f"err={error:.2f} "
            f"ctrl={control:.2f} "
            f"dt={dt:.3f}"
        )


# -----------------------------
# MAIN
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HeadingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
