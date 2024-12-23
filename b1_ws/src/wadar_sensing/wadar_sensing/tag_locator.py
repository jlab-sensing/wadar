import rclpy
from rclpy.node import Node
from inertial_sense_ros2.msg import DIDINS1
import tkinter as tk
import math
from wadar_sensing.globalPath import global_path

class TagLocator(Node):

    def __init__(self):
        super().__init__('tag_locator')
        self.gps_subscription = self.create_subscription(
            DIDINS1,
            'fake_ins1',
            self.gps_callback,
            10)

    def gps_callback(self, msg):
        self.latitude = msg.lla[0]
        self.longitude = msg.lla[1]
        self.heading = msg.theta[2]

        # self.get_logger().info(f'Latitude: {self.latitude}')
        # self.get_logger().info(f'Longitude: {self.longitude}')
        # self.get_logger().info(f'Heading: {self.heading}')

        tagLatitude = 37.3361663
        tagLongitude = -121.890591
        tagHeading = 0

        relativeAngle = global_path(self.latitude, self.longitude, self.heading, tagLatitude, tagLongitude)
        self.get_logger().info(f'Relative Angle: {relativeAngle}')

def main(args=None):
    rclpy.init(args=args)
    tag_locator = TagLocator()
    rclpy.spin(tag_locator)
    tag_locator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()