import rclpy
from rclpy.node import Node
from wadar_interfaces.msg import TagRelativeLocation
import math
import threading
import tkinter as tk

class TagUI(Node):
    def __init__(self):
        super().__init__('tag_ui')
        self.tag_subscription = self.create_subscription(
            TagRelativeLocation,
            'tag_relative_location',
            self.tag_callback,
            10)
        self.compass = None

    def tag_callback(self, msg):
        if msg is None:
            return
        self.relative_heading = msg.relative_heading
        self.alignment = msg.alignment
        self.distance = msg.distance

        if self.compass is not None:
            self.compass.update_heading(self.relative_heading)
            self.compass.update_distance(msg.distance)
            self.compass.update_alignment(self.alignment)

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
        heading_rad = math.radians(heading)
        x = 150 + 80 * math.sin(heading_rad)
        y = 150 - 80 * math.cos(heading_rad)
        self.coords(self.needle, 150, 150, x, y)
        if abs(heading - 360) < 5 or heading < 5:
            self.itemconfig(self.distance_light, fill="green")
        elif heading < 10 or heading > 350:
            self.itemconfig(self.distance_light, fill="yellow")
        else:
            self.itemconfig(self.distance_light, fill="red")

    def update_alignment(self, alignment):
        alignment_rad = math.radians(alignment)
        x_start = 150 + 80 * math.sin(alignment_rad)
        y_start = 400 - 80 * math.cos(alignment_rad)
        x_end = 150 - 80 * math.sin(alignment_rad)
        y_end = 400 + 80 * math.cos(alignment_rad)
        self.coords(self.alignment_needle, x_start, y_start, x_end, y_end)
        self.itemconfig(self.alignment_text, text="{:.4f} deg".format(alignment))
        if abs(alignment - 360) < 5 or alignment < 5:
            self.itemconfig(self.alignment_light, fill="green")
        elif alignment < 10 or alignment > 350:
            self.itemconfig(self.alignment_light, fill="yellow")
        else:
            self.itemconfig(self.alignment_light, fill="red")
    
    def update_distance(self, distance):
        self.itemconfig(self.distance_text, text="{:.4f} m".format(distance))

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