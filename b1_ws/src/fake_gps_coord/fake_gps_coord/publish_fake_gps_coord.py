import rclpy
import os
from rclpy.node import Node
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
        msg.lla = [37.7749, -122.4194, 0.0] 
        msg.ned = [0.0, 0.0, 0.0]   

        self.publisher_.publish(msg)


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