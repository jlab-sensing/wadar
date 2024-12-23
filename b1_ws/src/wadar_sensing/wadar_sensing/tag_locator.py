import rclpy
from rclpy.node import Node
from inertial_sense_ros2.msg import DIDINS1
from wadar_interfaces.msg import TagRelativeLocation
import math
from wadar_sensing.globalPath import global_path

import tkinter as tk

class TagLocator(Node):

    def __init__(self):
        super().__init__('tag_locator')
        self.gps_subscription = self.create_subscription(
            DIDINS1,
            'fake_ins1',
            self.ins_callback,
            10)
        self.tag_publisher = self.create_publisher(TagRelativeLocation, 'tag_relative_location', 10)

    def ins_callback(self, msg):
        self.latitude = msg.lla[0]
        self.longitude = msg.lla[1]
        self.heading = msg.theta[2]

        # self.get_logger().info(f'Latitude: {self.latitude}')
        # self.get_logger().info(f'Longitude: {self.longitude}')
        # self.get_logger().info(f'Heading: {self.heading}')

        tagLatitude = 37.3361663
        tagLongitude = -121.890591
        tagHeading = 0

        tagHeadingDiff = tagHeading - self.heading

        relativeAngle = global_path(self.latitude, self.longitude, self.heading, tagLatitude, tagLongitude)

        distance = math.sqrt((self.latitude - tagLatitude)**2 + (self.longitude - tagLongitude)**2) * 111139

        msg = TagRelativeLocation()
        msg.relative_heading_angle = relativeAngle
        msg.alignment_angle = tagHeadingDiff
        msg.distance = distance
        self.tag_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tag_locator = TagLocator()
    rclpy.spin(tag_locator)
    tag_locator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()