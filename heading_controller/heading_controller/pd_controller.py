import rclpy
from rclpy.node import Node
from serial_interfaces.msg import SensorData
from std_msgs.msg import Float64, Float64MultiArray, String
from motor_interfaces.msg import PWM

SCREW_START = 1500
SCREW_RANGE = 400
SCREW_LEFT_CONSTANT = 1350

CRAB_START = 1500
CRAB_RANGE = 400
CRAB_LEFT_CONSTANT = 1350



class HeadingController(Node):

    def __init__(self):
        super().__init__('heading_controller')

        # -----------------------------
        # SUB / PUB
        # -----------------------------
        self.create_subscription(SensorData, '/sensor_data', self.sensor_cb, 10)
        self.create_subscription(String, '/terrain_id', self.terrain_cb, 10)
        self.create_subscription(Float64, '/desired_yaw', self.desired_yaw_cb, 10)
        self.create_subscription(Float64MultiArray, '/pd_gains', self.gains_cb, 10)

        self.publisher = self.create_publisher(PWM, '/motor_pwm', 10)

        # -----------------------------
        # CONTROL PARAMETERS
        # -----------------------------
        self.desired_yaw = 0.0
        self.kp = 0.6
        self.kd = 0.15
        self.terrain = "dry_sand"

        # -----------------------------
        # STATE
        # -----------------------------
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        self.get_logger().info("Heading Controller Started")

    # -----------------------------
    # CALLBACKS
    # -----------------------------
    def terrain_cb(self, msg):
        self.terrain = msg.data

    def desired_yaw_cb(self, msg):
        self.desired_yaw = msg.data

    def gains_cb(self, msg):
        if len(msg.data) >= 2:
            self.kp = msg.data[0]
            self.kd = msg.data[1]

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
    def sensor_cb(self, msg):

        # -------------------------
        # 1. TIME STEP
        # -------------------------
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if dt <= 0.0:
            dt = 0.05

        # -------------------------
        # 2. ERROR (shortest path)
        # -------------------------
        error = self.wrap_angle(self.desired_yaw - msg.yaw)

        # -------------------------
        # 3. DERIVATIVE (dt-aware)
        # -------------------------
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # -------------------------
        # 4. PD CONTROL
        # -------------------------
        control = self.kp * error + self.kd * derivative

        # -------------------------
        # 5. SATURATION
        # -------------------------
        control = min(1.0, max(0.0, control))

        # -------------------------
        # 6. MOTOR MIXING
        # -------------------------
        pwm_msg = PWM()

        if self.terrain == "wet_sand":
            # screw mode: both sides scaled around midpoint
            pwm_msg.left_pwm = float(SCREW_LEFT_CONSTANT)
            pwm_msg.right_pwm = float(SCREW_START + control * SCREW_RANGE)
        else:
            # crab mode: right side constant, left side adjusted by PD
            pwm_msg.left_pwm = float(CRAB_LEFT_CONSTANT)
            pwm_msg.right_pwm = float(CRAB_START - control * CRAB_RANGE)
            

        # -------------------------
        # 7. PUBLISH PWM
        # -------------------------
        self.publisher.publish(pwm_msg)

        # -------------------------
        # DEBUG
        # -------------------------
        self.get_logger().info(
            f"yaw={msg.yaw:.2f} "
            f"err={error:.2f} "
            f"ctrl={control:.2f} "
            f"dt={dt:.3f} "
            f"mode={self.terrain}"
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