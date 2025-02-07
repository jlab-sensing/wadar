import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from inertial_sense_ros2.msg import DIDINS1
from std_msgs.msg import Header
import math


class GPSIMUEmulator(Node):
    def __init__(self):
        super().__init__('gps_imu_emulator')
        self.ins_pub = self.create_publisher(DIDINS1, 'fake_ins1', 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.longitude = None
        self.imu_data = None

    def gps_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.publish_ins_data()

    def imu_callback(self, data):
        self.imu_data = data
        self.heading = self.calculate_heading(data.orientation)
        self.publish_ins_data()

    def calculate_heading(self, orientation):
        # Convert quaternion to euler angles
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        heading = math.atan2(siny_cosp, cosy_cosp)
        return heading

    def publish_ins_data(self):
        if self.longitude is not None and self.imu_data is not None:
            ins_msg = DIDINS1()
            ins_msg.header = Header()
            ins_msg.header.stamp = self.get_clock().now().to_msg()
            ins_msg.header.frame_id = "did_ins1"

            ins_msg.week = 0
            ins_msg.time_of_week = 0.0 

            ins_msg.ins_status = 1
            ins_msg.hdw_status = 1

            ins_msg.theta = [0.0, 0.0, self.heading]
            ins_msg.uvw = [0.0, 0.0, 0.0]    
            ins_msg.lla = [self.latitude, self.longitude, 0.0]
            ins_msg.ned = [0.0, 0.0, 0.0]   

            self.ins_pub.publish(ins_msg)


def main(args=None):
    rclpy.init(args=args)
    emulator = GPSIMUEmulator()
    rclpy.spin(emulator)
    emulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()