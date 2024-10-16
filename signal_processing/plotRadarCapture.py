#  File:   plotRadarCapture.py
#  Author: ericdvet
# 
#  Plot the 3D FT of the captured radar data and the FT of the isolated tag

import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import argparse

# Set up argument parser
parser = argparse.ArgumentParser(description='Plot radar capture data.')
parser.add_argument('tagFT', type=str, help='CSV file for tag FT data')
parser.add_argument('captureFT', type=str, help='CSV file for capture FT data')
args = parser.parse_args()

# Read tagFT data
tagFT = []
with open(args.tagFT, mode='r') as file:
    reader = csv.reader(file)
    for row in reader:
        tagFT.append(float(row[0]))

plt.figure()
plt.plot(tagFT, linestyle='-', color='b')
plt.title(f"80 Hz Isolated - {args.tagFT}")
plt.xlabel("Range Bins")
plt.ylabel("Magnitude")
plt.savefig("tagFT.png")

# Read captureFT data
data = np.loadtxt(args.captureFT, delimiter=",", skiprows=0)

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

fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(xMat, yMat, zMat, cmap='viridis', edgecolor='none')
ax.set_xlabel('Range Bins')
ax.set_ylabel('Frequency')
ax.set_zlabel('Magnitude')
ax.set_title(f'FT bins - {args.captureFT}')
plt.savefig("captureFT.png")

# # Last plot
# _, frame_count = data.shape
# plt.figure()

# for j in range(1, frame_count):
#     plt.plot(np.abs(data[:, j]), label=f'Frame {j}')

# plt.xlabel('Range Bins')
# plt.ylabel('Magnitude')
# plt.title("Capture - FT of all peak bins")

# # Save and show the plot
# plt.savefig("peak_bins.png")
# plt.show()

plt.show()
