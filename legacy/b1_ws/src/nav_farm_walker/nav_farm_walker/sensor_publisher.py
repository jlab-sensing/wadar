import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from inertial_sense_ros2.msg import DIDINS2, GPS

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')

        self.imu_sub = self.create_subscription(DIDINS2, '/ins_quat_uvw_lla', self.imu_callback, 10) 
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)  

        self.gps_sub = self.create_subscription(GPS, '/gps1/pos_vel', self.gps_callback, 10)  
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom_ins_enu', self.odom_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)  

    def imu_callback(self, data):
        self.publish_imu_data(data)

    def publish_imu_data(self, data):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        quat_ned = [float(data.qn2b[1]), float(data.qn2b[2]), float(data.qn2b[3]), float(data.qn2b[0])]
        v_ned = [quat_ned[0], quat_ned[1], quat_ned[2]]
        v_enu = [v_ned[1], v_ned[0], -v_ned[2]]
        quat_enu = [v_enu[0], v_enu[1], v_enu[2], quat_ned[3]]

        imu_msg.orientation.x = quat_enu[0]
        imu_msg.orientation.y = quat_enu[1]
        imu_msg.orientation.z = quat_enu[2]
        imu_msg.orientation.w = quat_enu[3]

        imu_msg.angular_velocity.x = float(data.uvw[0])
        imu_msg.angular_velocity.y = float(data.uvw[1])
        imu_msg.angular_velocity.z = float(data.uvw[2])
        # only doing this because the imu data published isn't correct and we don't need acceleration anyway
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        self.imu_pub.publish(imu_msg)
    
    def gps_callback(self, data):
        self.publish_gps_data(data)

    def publish_gps_data(self, data):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"
        gps_msg.latitude = data.latitude
        gps_msg.longitude = data.longitude
        gps_msg.altitude = data.altitude
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_pub.publish(gps_msg)

    def odom_callback(self, data):
        self.publish_odom_data(data)

    def publish_odom_data(self, data):
        odom_msg = Odometry()
        odom_msg = data
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()