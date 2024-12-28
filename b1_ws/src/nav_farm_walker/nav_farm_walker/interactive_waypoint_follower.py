import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from nav_farm_walker.utils.gps_utils import latLonYaw2Geopose
from geographic_msgs.msg import GeoPoseStamped 


class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)
        
        self.wp_publisher = self.create_publisher(GeoPoseStamped, 'current_waypoint', 10) 

    def mapviz_wp_cb(self, msg: PointStamped):
        """
        clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
        """
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point from mapviz that is not in wgs84 frame. This is not a gps point and won't be followed")
            return

        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wp = latLonYaw2Geopose(msg.point.y, msg.point.x)
        
        geo_pose_stamped = GeoPoseStamped()
        geo_pose_stamped.pose = wp
        geo_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.wp_publisher.publish(geo_pose_stamped) 

        self.navigator.followGpsWaypoints([wp])
        if self.navigator.isTaskComplete():
            self.get_logger().info("wps completed successfully")


def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    rclpy.spin(gps_wpf)


if __name__ == "__main__":
    main()
