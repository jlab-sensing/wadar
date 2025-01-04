import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from inertial_sense_ros2.msg import DIDINS2, DIDINS4, GPS

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10) 
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
        imu_msg.orientation.x = data.orientation.x
        imu_msg.orientation.y = data.orientation.y
        imu_msg.orientation.z = data.orientation.z
        imu_msg.orientation.w = data.orientation.w
        imu_msg.angular_velocity.x = data.angular_velocity.x
        imu_msg.angular_velocity.y = data.angular_velocity.y
        imu_msg.angular_velocity.z = data.angular_velocity.z
        imu_msg.linear_acceleration.x = data.linear_acceleration.x
        imu_msg.linear_acceleration.y = data.linear_acceleration.y
        imu_msg.linear_acceleration.z = data.linear_acceleration.z
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