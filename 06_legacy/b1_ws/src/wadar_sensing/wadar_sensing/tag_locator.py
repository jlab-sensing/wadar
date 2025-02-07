import rclpy
from rclpy.node import Node
from inertial_sense_ros2.msg import DIDINS1
from wadar_interfaces.msg import TagRelativeLocation
import math
from wadar_sensing.globalPath import global_path
class TagLocator(Node):

    def __init__(self):
        super().__init__('tag_locator')
        self.latitude = None
        self.longitude = None
        self.heading = None

        # Tentatively hardcoded tag location
        self.tag_latitude = 36.982271
        self.tag_longitude = -122.053967
        self.tag_heading = 0

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

        if self.heading is None or self.latitude is None or self.longitude is None:
            self.get_logger().warn('Heading, latitude, or longitude is None, skipping calculation.')
            return

        tag_heading_difference = self.tag_heading - self.heading

        relative_angle = global_path(self.latitude, self.longitude, self.heading, self.tag_latitude, self.tag_longitude)

        # Haversine formula to calculate the distance
        R = 6371000  # Radius of the Earth in meters
        lat1 = math.radians(self.latitude)
        lon1 = math.radians(self.longitude)
        lat2 = math.radians(self.tag_latitude)
        lon2 = math.radians(self.tag_longitude)

        # Differences in coordinates
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Haversine formula
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Distance in meters
        distance = R * c

        tag_msg = TagRelativeLocation()
        tag_msg.relative_heading = relative_angle
        tag_msg.alignment = tag_heading_difference
        tag_msg.distance = distance
        self.tag_publisher.publish(tag_msg)

def main(args=None):
    """
    Initialize and spin the TagLocator node, which subscribes to GPS data and publishes the relative location of a tag.

    Args:
        args (list, optional): List of command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    tag_locator = TagLocator()
    try:
        rclpy.spin(tag_locator)
    finally:
        tag_locator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()