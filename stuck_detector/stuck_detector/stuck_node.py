import rclpy
from rclpy.node import Node
from serial_interfaces.msg import SensorData
from std_msgs.msg import String
import numpy as np
from collections import deque

class StuckDetector(Node):
    def __init__(self):
        super().__init__('stuck_detector')

        self.subscription = self.create_subscription(
            SensorData,
            '/sensor_data',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/stuck_status',
            10
        )

        # --- buffers ---
        self.window_size = 20
        self.buffer = deque(maxlen=self.window_size)


        # thresholds (we’ll tune later)
        self.rpm_thresh = 20
        self.acc_thresh = 0.2
        self.current_thresh = 2.0

    # =========================================================
    # ===== THIS IS WHERE STEPS 6+ GO ==========================
    # =========================================================

    def callback(self, msg):

        # ===== STEP 6: EXTRACT RAW FEATURES =====
        rpm = (msg.rpm_left + msg.rpm_right) / 2.0

        acc_mag = np.sqrt(
            msg.acc_x**2 +
            msg.acc_y**2
        )

        current = (msg.curr_m_left + msg.curr_m_right) / 2.0

        sinkage = msg.tof_mm


        # ===== STEP 7: ADD TO BUFFER =====
        self.buffer.append([rpm, acc_mag, current, sinkage])

        if len(self.buffer) < self.window_size:
            return  # wait until buffer fills


        # ===== STEP 8: COMPUTE WINDOW STATISTICS =====
        data = np.array(self.buffer)

        rpm_mean = np.mean(data[:, 0])
        acc_mean = np.mean(data[:, 1])
        current_mean = np.mean(data[:, 2])
        sinkage_mean = np.mean(data[:, 3])


        # ===== STEP 9: RESIDUAL / CONSISTENCY LOGIC =====
        high_rpm = rpm_mean > self.rpm_thresh
        low_motion = acc_mean < self.acc_thresh
        high_current = current_mean > self.current_thresh
        high_sinkage = sinkage_mean > 30  # tune later


        stuck = (
            (high_rpm and low_motion and high_current)
            or (high_sinkage and high_rpm)
        )


        # ===== STEP 10: PUBLISH RESULT =====
        msg_out = String()

        if stuck:
            msg_out.data = "STUCK"
        else:
            msg_out.data = "FREE"

        self.publisher.publish(msg_out)

        self.get_logger().info(
            f"Stuck={msg_out.data} | "
            f"rpm={rpm_mean:.1f} "
            f"acc={acc_mean:.2f} "
            f"current={current_mean:.2f}"
        )


# ===== STEP 11: MAIN FUNCTION =====
def main(args=None):
    rclpy.init(args=args)
    node = StuckDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
