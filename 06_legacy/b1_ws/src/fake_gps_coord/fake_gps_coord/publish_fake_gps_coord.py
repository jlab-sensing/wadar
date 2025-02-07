import rclpy
import os
from rclpy.node import Node
from math import sin, cos
from inertial_sense_ros2.msg import DIDINS1
from std_msgs.msg import Header

class GpsNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(DIDINS1, 'fake_ins1', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = DIDINS1()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.header.frame_id = "did_ins1"

        msg.week = 0
        msg.time_of_week = 0.0 

        msg.ins_status = 1
        msg.hdw_status = 1

        msg.theta = [0.0, 0.0, 0.0]  
        msg.uvw = [0.0, 0.0, 0.0]    
        msg.lla = [36.982505, -122.055467, 0.0] 
        msg.ned = self.lla_to_ned(msg.lla)

        self.publisher_.publish(msg)

    def lla_to_ned(self, lla):
        lat0, lon0, alt0 = 36.982505, -122.055467, 0.0 # i have 0 fucking clue what these values are supposed to be

        lat = lla[0] * (3.141592653589793 / 180.0)
        lon = lla[1] * (3.141592653589793 / 180.0)
        lat0 = lat0 * (3.141592653589793 / 180.0)
        lon0 = lon0 * (3.141592653589793 / 180.0)

        a = 6378137.0  
        f = 1 / 298.257223563  
        e2 = 2 * f - f ** 2 

        N = a / (1 - e2 * (sin(lat0) ** 2)) ** 0.5

        x0 = (N + alt0) * cos(lat0) * cos(lon0)
        y0 = (N + alt0) * cos(lat0) * sin(lon0)
        z0 = (N * (1 - e2) + alt0) * sin(lat0)

        N = a / (1 - e2 * (sin(lat) ** 2)) ** 0.5
        x = (N + lla[2]) * cos(lat) * cos(lon)
        y = (N + lla[2]) * cos(lat) * sin(lon)
        z = (N * (1 - e2) + lla[2]) * sin(lat)

        ned_x = -(x - x0) * sin(lat0) * cos(lon0) - (y - y0) * sin(lat0) * sin(lon0) + (z - z0) * cos(lat0)
        ned_y = -(x - x0) * sin(lon0) + (y - y0) * cos(lon0)
        ned_z = -(x - x0) * cos(lat0) * cos(lon0) - (y - y0) * cos(lat0) * sin(lon0) - (z - z0) * sin(lat0)

        return [ned_x, ned_y, ned_z]


def main(args=None):
    rclpy.init(args=args)

    gps_node = GpsNode()

    rclpy.spin(gps_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()