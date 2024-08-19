#  File:   plotRadarCapture.py
#  Author: ericdvet
# 
#  Plot the 3D FT of the captured radar data and the FT of the isolated tag
#  Created on 2021-07-07

import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

csv_filename = "tagFT.csv"
tagFT = []

with open(csv_filename, mode='r') as file:
    reader = csv.reader(file)
    for row in reader:
        tagFT.append(float(row[0]))

plt.figure()
plt.plot(tagFT, linestyle='-', color='b')
plt.title("80 Hz Isolated")
plt.xlabel("Range Bins")
plt.ylabel("Magnitude")
plt.savefig("tagFT.png")

data = np.loadtxt("captureFT.csv", delimiter=",", skiprows=0)

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
ax.set_title('FT bins')
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
