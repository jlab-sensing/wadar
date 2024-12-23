class Compass(tk.Canvas):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.width = 500
        self.height = 500
        self.create_oval(50, 50, 250, 250, fill="white")
        self.create_line(150, 150, 150, 50, fill="black")
        self.create_text(150, 45, text="TAG")
        self.needle = self.create_line(150, 150, 150, 50, fill="red", width=2)

    def update_heading(self, heading):
        # Convert heading to radians
        heading_rad = math.radians(heading)

        # Calculate needle coordinates
        x = 150 + 80 * math.sin(heading_rad)
        y = 150 - 80 * math.cos(heading_rad)

        # Update needle
        self.coords(self.needle, 150, 150, x, y)

def start_tkinter():
    root = tk.Tk()
    compass = Compass(root)
    compass.pack()

    # Simulate heading update
    compass.update_heading(45)

    root.mainloop()