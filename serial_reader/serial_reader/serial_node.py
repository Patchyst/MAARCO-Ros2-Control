import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from serial_interfaces.msg import SensorData

import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.publisher_ = self.create_publisher(SensorData, 'sensor_data', 10)
        #self.publisher_ = self.create_publisher(String, 'sensor_data', 10)

        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            timeout=1
        )

        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

        if line:
            self.get_logger().info(f"RAW: {line}")

            parts = line.split(',')

            if len(parts) == 23 and parts[0] == "Ard":
                try:
                    msg = SensorData()

                    
                    msg.timestamp = int(float(parts[1]))

                    msg.volt_b_left = float(parts[2])
                    msg.curr_b_left = float(parts[3])
                    msg.volt_b_right = float(parts[4])
                    msg.curr_b_right = float(parts[5])

                    msg.curr_m_left = float(parts[6])
                    msg.curr_m_right = float(parts[7])
                
                    msg.roll = -float(parts[9]) #roll = -euler_y = parts[9]
                    msg.pitch = float(parts[10]) #pitch = euler_z = parts[10]
                    msg.yaw = -float(parts[8]) #yaw = -euler_x = parts[8]
                
                    msg.acc_x = float(parts[11])
                    msg.acc_y = float(parts[12])
                    msg.acc_z = float(parts[13])
                
                    msg.roll_rate = -float(parts[15]) #roll_rate = parts[15]
                    msg.pitch_rate = float(parts[16]) #pitch_rate = parts[16]
                    msg.yaw_rate = -float(parts[14]) #yaw_rate = -parts[14]

                    msg.sonar_mm = float(parts[17])

                    msg.tof_mm = float(parts[18])

                    msg.rpm_left = float(parts[19])
                    msg.rpm_right = float(parts[20])
                    msg.nrot_left = float(parts[21])
                    msg.nrot_right = float(parts[22])

                    self.publisher_.publish(msg)

                    self.get_logger().info(f"Yaw: {msg.yaw}")

                except Exception as e:
                    self.get_logger().error(f"Parse error: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
