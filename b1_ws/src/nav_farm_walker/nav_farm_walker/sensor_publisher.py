import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from inertial_sense_ros2.msg import DIDINS1
import math

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/gps', 10)
        self.ins_sub = self.create_subscription(DIDINS1, '/fake_ins1', self.ins_callback, 10)  # Subscription to fake_ins

    def ins_callback(self, data):
        self.publish_imu_data(data)
        self.publish_gps_data(data)
        self.publish_odometry_data(data)

    def publish_imu_data(self, data):
        imu_msg = Imu()
        imu_msg.header = data.header
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.angular_velocity.x = float(data.uvw[0])
        imu_msg.angular_velocity.y = float(data.uvw[1])
        imu_msg.angular_velocity.z = float(data.uvw[2])
        imu_msg.linear_acceleration.x = float(data.ned[0])
        imu_msg.linear_acceleration.y = float(data.ned[1])
        imu_msg.linear_acceleration.z = float(data.ned[2])
        # Assuming theta contains roll, pitch, yaw
        imu_msg.orientation = self.euler_to_quaternion(data.theta[0], data.theta[1], data.theta[2])

        self.imu_pub.publish(imu_msg)

    def publish_gps_data(self, data):
        gps_msg = NavSatFix()
        gps_msg.header = data.header
        gps_msg.header.frame_id = "gps_link"
        gps_msg.latitude = data.lla[0]
        gps_msg.longitude = data.lla[1]
        gps_msg.altitude = data.lla[2]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(gps_msg)

    def publish_odometry_data(self, data):
        odom_msg = Odometry()
        odom_msg.header = data.header
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.orientation = self.euler_to_quaternion(data.theta[0], data.theta[1], data.theta[2])
        odom_msg.pose.pose.position.x = float(data.ned[0])
        odom_msg.pose.pose.position.y = float(data.ned[1])
        odom_msg.pose.pose.position.z = float(data.ned[2])

        self.odom_pub.publish(odom_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        quaternion = Quaternion()
        quaternion.x = qx
        quaternion.y = qy
        quaternion.z = qz
        quaternion.w = qw
        return quaternion

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()