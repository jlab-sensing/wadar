import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import tkinter as tk
import math
from wadar_sensing.globalPath import global_path

class TagLocator(Node):

    def __init__(self):
        super().__init__('tag_locator')
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        self.gps_subscription 
        self.root = tk.Tk()
        self.gui = TagLocatorGUI(master=self.root)
        # self.root.after(0, self.update_gui)
        # self.root.mainloop()

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        print('Longitude, Latitude, Altitude: %f, %f, %f' % (self.longitude, self.latitude, self.altitude))

        tagLatitude = 20.593683
        tagLongitude = 78.962883
        tagAltitude = 1.15

        tagDiffLatitude = tagLatitude - self.latitude
        tagDiffLongitude = tagLongitude - self.longitude
        tagDiffAltitude = tagAltitude - self.altitude

        angle = math.degrees(math.atan2(tagDiffLongitude, tagDiffLatitude))
        print('Angle: %f' % angle)
        self.gui.update_pin(angle)

    def update_gui(self):
        self.root.update_idletasks()
        self.root.update()
        self.root.after(100, self.update_gui)

def main(args=None):
    rclpy.init(args=args)
    tag_locator = TagLocator()
    rclpy.spin(tag_locator)
    tag_locator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

class TagLocatorGUI(tk.Canvas):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        self.canvas = tk.Canvas(self, width=400, height=400, bg='white')
        self.canvas.pack()
        self.draw_compass()

    def draw_compass(self):
        self.canvas.create_oval(50, 50, 350, 350)
        self.canvas.create_line(200, 50, 200, 350, arrow=tk.LAST)
        self.canvas.create_text(200, 30, text='N')
        self.pin = self.canvas.create_line(200, 200, 200, 100, arrow=tk.LAST, fill='red')

    def update_pin(self, angle):
        x = 200 + 100 * math.sin(math.radians(angle))
        y = 200 - 100 * math.cos(math.radians(angle))
        self.canvas.coords(self.pin, 200, 200, x, y)