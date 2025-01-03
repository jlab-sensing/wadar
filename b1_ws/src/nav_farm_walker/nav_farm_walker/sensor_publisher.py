import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from inertial_sense_ros2.msg import DIDINS2, GPS

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/gps', 10)
        self.odom_pub_additional = self.create_publisher(Odometry, '/odom', 10)  # Additional publisher for /odom
        self.ins_sub = self.create_subscription(DIDINS2, '/ins_quat_uvw_lla', self.ins_callback, 10)  # Subscription to DIDINS2
        self.gps_sub = self.create_subscription(GPS, '/gps1/pos_vel', self.gps_callback, 10)  # Subscription to GPS data

    def ins_callback(self, data):
        self.publish_imu_data(data)
        self.publish_odometry_data(data)

    def gps_callback(self, data):
        self.publish_gps_data(data)

    def publish_imu_data(self, data):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.orientation.x = float(data.qn2b[0])
        imu_msg.orientation.y = float(data.qn2b[1])
        imu_msg.orientation.z = float(data.qn2b[2])
        imu_msg.orientation.w = float(data.qn2b[3])
        imu_msg.angular_velocity.x = float(data.uvw[0])
        imu_msg.angular_velocity.y = float(data.uvw[1])
        imu_msg.angular_velocity.z = float(data.uvw[2])
        imu_msg.linear_acceleration.x = float(data.uvw[0])
        imu_msg.linear_acceleration.y = float(data.uvw[1])
        imu_msg.linear_acceleration.z = float(data.uvw[2])

        self.imu_pub.publish(imu_msg)

    def publish_gps_data(self, data):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"
        gps_msg.latitude = data.latitude
        gps_msg.longitude = data.longitude
        gps_msg.altitude = data.altitude
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(gps_msg)

    def publish_odometry_data(self, data):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.orientation.x = float(data.qn2b[0])
        odom_msg.pose.pose.orientation.y = float(data.qn2b[1])
        odom_msg.pose.pose.orientation.z = float(data.qn2b[2])
        odom_msg.pose.pose.orientation.w = float(data.qn2b[3])
        odom_msg.pose.pose.position.x = float(data.lla[0])
        odom_msg.pose.pose.position.y = float(data.lla[1])
        odom_msg.pose.pose.position.z = float(data.lla[2])

        self.odom_pub.publish(odom_msg)
        self.odom_pub_additional.publish(odom_msg)  # Publish to /odom

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()