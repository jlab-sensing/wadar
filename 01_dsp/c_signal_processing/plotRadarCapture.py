#!/usr/bin/env python3
#  File:   plotRadarCapture.py
#  Author: ericdvet
# 
#  Plot the 3D FT of the captured radar data and the FT of the isolated tag

import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import filedialog, Button

def load_files():
    root = tk.Tk()
    root.withdraw()

    tagFT_file = filedialog.askopenfilename(title="Select tag FT CSV file", filetypes=[("CSV files", "*.csv")])
    captureFT_file = filedialog.askopenfilename(title="Select capture FT CSV file", filetypes=[("CSV files", "*.csv")])

    if not tagFT_file or not captureFT_file:
        print("Both files need to be selected.")
        return

    plot_data(tagFT_file, captureFT_file)

def plot_data(tagFT_file, captureFT_file):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))

    # Read tagFT data
    tagFT = []
    with open(tagFT_file, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            tagFT.append(float(row[0]))

    ax1.plot(tagFT, linestyle='-', color='b')
    ax1.set_title(f"Tag Bin Isolated - {tagFT_file}")
    ax1.set_xlabel("Range Bins")
    ax1.set_ylabel("Magnitude")

    # Read captureFT data
    data = np.loadtxt(captureFT_file, delimiter=",", skiprows=0)

    rows, cols = data.shape
    processed_frames = cols // 100

    # Processing data 
    data[:, 0:2] = 1  # First 2 frames are noisy
    data[:, 0:processed_frames] = 1
    data[:, -processed_frames:] = 1 

    x = np.arange(1, rows + 1)  # Range bins (1 to number of rows)
    y = np.arange(1, cols + 1) / cols * 200  # Frequency
    xMat, yMat = np.meshgrid(x, y)
    zMat = np.abs(data.T) 

    ax2 = fig.add_subplot(122, projection='3d')
    surf = ax2.plot_surface(xMat, yMat, zMat, cmap='viridis', edgecolor='none')
    ax2.set_xlabel('Range Bins')
    ax2.set_ylabel('Frequency')
    ax2.set_zlabel('Magnitude')
    ax2.set_title(f'FT bins - {captureFT_file}')

    root = tk.Tk()
    root.title("Options")

    reselect_button = Button(root, text="Reselect Files", command=load_files)
    reselect_button.pack()

    exit_button = Button(root, text="Exit", command=root.quit)
    exit_button.pack()

    plt.show()
    root.mainloop()

if __name__ == "__main__":
    load_files()
