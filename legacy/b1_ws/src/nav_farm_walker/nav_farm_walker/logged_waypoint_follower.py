import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time

from nav_farm_walker.utils.gps_utils import latLonYaw2Geopose
from geographic_msgs.msg import GeoPoseStamped  # Import the message type
from rclpy.node import Node  # Import Node


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWpCommander(Node):
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        super().__init__('gps_wp_commander')
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.wp_publisher = self.create_publisher(GeoPoseStamped, 'current_waypoint', 10)  # Initialize the publisher

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wps = self.wp_parser.get_wps()
        for wp in wps:
            geo_pose_stamped = GeoPoseStamped()
            geo_pose_stamped.pose = wp
            geo_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.wp_publisher.publish(geo_pose_stamped)  # Publish the current waypoint
            self.navigator.followGpsWaypoints([wp])
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)
        print("wps completed successfully")


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "gps_follower"), "config", "demo_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()
