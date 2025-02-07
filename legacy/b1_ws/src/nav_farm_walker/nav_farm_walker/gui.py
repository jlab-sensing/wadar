import rclpy
from rclpy.node import Node
from wadar_interfaces.msg import TagRelativeLocation
from inertial_sense_ros2.msg import DIDINS2
import math
import threading
import tkinter as tk
from nav_msgs.msg import Path

from geographic_msgs.msg import GeoPoseStamped  # Import the message type
from rclpy.node import Node  # Import Node

class TagUI(Node):
    def __init__(self):
        super().__init__('tag_ui')
        self.gps_subscription = self.create_subscription(
            DIDINS2,
            'ins_quat_uvw_lla',
            self.ins_callback,
            10)
        self.local_plan_subscription = self.create_subscription(
            Path,
            'local_plan',
            self.local_plan_callback,
            10)
        self.waypoint_subscription = self.create_subscription(
            GeoPoseStamped,
            'current_waypoint',
            self.waypoint_callback,
            10)
        self.compass = None
        self.tag_latitude = 0
        self.tag_longitude = 0
        self.tag_heading = 0
        self.latitude = 0
        self.longitude = 0
        self.heading = 0

    def ins_callback(self, msg):
        if msg.lla is None:
            self.get_logger().warn('msg.lla None, skipping calculation.')
            return

        self.latitude = msg.lla[0]
        self.longitude = msg.lla[1]
        
        quat_ned = [float(msg.qn2b[1]), float(msg.qn2b[2]), float(msg.qn2b[3]), float(msg.qn2b[0])]
        v_ned = [quat_ned[0], quat_ned[1], quat_ned[2]]
        v_enu = [v_ned[1], v_ned[0], -v_ned[2]]
        quat_enu = [v_enu[0], v_enu[1], v_enu[2], quat_ned[3]]
        euler_enu = [0, 0, math.atan2(2*(quat_enu[3]*quat_enu[2] + quat_enu[0]*quat_enu[1]), 1 - 2*(quat_enu[1]**2 + quat_enu[2]**2))]

        R = 6371000  # Radius of the Earth in meters
        lat1 = math.radians(self.latitude)
        lon1 = math.radians(self.longitude)
        lat2 = math.radians(self.tag_latitude)
        lon2 = math.radians(self.tag_longitude)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c

        if self.compass is not None:
            self.compass.update_distance(distance)
            self.compass.update_alignment(self.tag_heading - euler_enu[2])
        else:
            self.get_logger().warn('Compass is None, skipping update.')

    def local_plan_callback(self, msg):
        if not msg.poses:
            self.get_logger().warn('msg.poses is empty, skipping calculation.')
            return

        self.x = msg.poses[0].pose.orientation.x
        self.y = msg.poses[0].pose.orientation.y
        self.z = msg.poses[0].pose.orientation.z
        self.w = msg.poses[0].pose.orientation.w

        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        heading = math.atan2(siny_cosp, cosy_cosp)
        relative_heading = heading - self.heading

        if self.compass is not None:
            self.compass.update_heading(heading - math.pi/2)
        else:
            self.get_logger().warn('Compass is None, skipping update.')

    def waypoint_callback(self, msg):
        self.tag_latitude = msg.pose.position.latitude
        self.tag_longitude = msg.pose.position.longitude
        self.tag_x = msg.pose.orientation.x
        self.tag_y = msg.pose.orientation.y
        self.tag_z = msg.pose.orientation.z
        self.tag_w = msg.pose.orientation.w

        siny_cosp = 2 * (self.tag_w * self.tag_z + self.tag_x * self.tag_y)
        cosy_cosp = 1 - 2 * (self.tag_y * self.tag_y + self.tag_z * self.tag_z)
        self.tag_heading = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(f'Updated tag location: lat={self.tag_latitude}, lon={self.tag_longitude}, heading={self.tag_heading}')

class Compass(tk.Canvas):
    def __init__(self, master, **kwargs):
        super().__init__(master, width=600, height=600, **kwargs)
        self.create_text(150, 20, text="Directions", anchor="center", font=("Arial", 12))
        self.create_oval(50, 50, 250, 250, fill="white")
        self.create_line(150, 150, 150, 50, fill="black")
        self.create_text(150, 45, text="TAG", anchor="center", font=("Arial", 12))
        self.needle = self.create_line(150, 150, 150, 50, fill="red", width=3)
        self.distance_label = self.create_text(400, 50, text="Distance to tag:", anchor="w", font=("Arial", 10))
        self.distance_text = self.create_text(400, 70, text="0.0000 m", anchor="w", font=("Arial", 10))
        self.create_text(150, 280, text="Tag Alignment", anchor="center", font=("Arial", 12))
        self.create_oval(50, 300, 250, 500, fill="white")
        self.create_line(150, 300, 150, 500, fill="black")
        self.alignment_needle = self.create_line(150, 300, 150, 500, fill="blue", width=3)
        self.create_text(400, 100, text="Alignment", anchor="w", font=("Arial", 10))
        self.alignment_text = self.create_text(400, 120, text="0.0000 deg", anchor="w", font=("Arial", 10))
        self.distance_light = self.create_oval(400, 150, 420, 170, fill="red")
        self.alignment_light = self.create_oval(400, 180, 420, 200, fill="red")
        self.create_text(430, 160, text="Distance Indicator", anchor="w", font=("Arial", 10))
        self.create_text(430, 190, text="Alignment Indicator", anchor="w", font=("Arial", 10))

    def update_heading(self, heading):
        x = 150 + 80 * math.sin(heading)
        y = 150 - 80 * math.cos(heading)
        self.coords(self.needle, 150, 150, x, y)

    def update_alignment(self, alignment):
        alignment_deg = math.degrees(alignment)
        x_start = 150 + 80 * math.sin(alignment)
        y_start = 400 - 80 * math.cos(alignment)
        x_end = 150 - 80 * math.sin(alignment)
        y_end = 400 + 80 * math.cos(alignment)
        self.coords(self.alignment_needle, x_start, y_start, x_end, y_end)
        self.itemconfig(self.alignment_text, text="{:.4f} deg".format(alignment_deg))
        if (alignment_deg < 10 and alignment_deg >= 0) or (alignment_deg > 350 and alignment_deg <= 360):
            self.itemconfig(self.alignment_light, fill="green")
        elif (alignment_deg < 20 and alignment_deg >= 10) or (alignment_deg > 340 and alignment_deg <= 350):
            self.itemconfig(self.alignment_light, fill="yellow")
        else:
            self.itemconfig(self.alignment_light, fill="red")
    
    def update_distance(self, distance):
        self.itemconfig(self.distance_text, text="{:.4f} m".format(distance))
        if abs(distance) < 1:
            self.itemconfig(self.distance_light, fill="green")
        elif distance < 5:
            self.itemconfig(self.distance_light, fill="yellow")
        else:
            self.itemconfig(self.distance_light, fill="red")

def start_tkinter(tag_ui):
    root = tk.Tk()
    root.title("Tag Locator 0.1.0b")
    root.geometry("800x600")  
    compass = Compass(root)
    compass.pack(expand=True, fill=tk.BOTH) 
    tag_ui.compass = compass
    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    tag_ui = TagUI()

    tkinter_thread = threading.Thread(target=start_tkinter, args=(tag_ui,)) # so it doesn't block the subscriber
    tkinter_thread.start()

    rclpy.spin(tag_ui)
    tag_ui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()