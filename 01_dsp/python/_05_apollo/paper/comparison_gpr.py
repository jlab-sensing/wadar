import matplotlib.pyplot as plt

labels = ['Our Radar', 'Alzubaidi et al. 2024', 'Freeland et al. 2008', 'Wang et al. 2016']
depth_00 = [33.0569, 41.3211, 66.1138, 33.0569]     # loam @ 0.0% moisture
depth_20 = [0.2801, 0.3501, 0.5602, 0.2801]     # loam @ 20.0% moisture 
depth_40 = [0.2090, 0.2612, 0.4179, 0.2090]     # loam @ 40.0% moisture

c = 299792458  # m/s
resolution = [0.004, c/(2*400e6), c/(2*1.6e9), 0.15]

fig, ax = plt.subplots(figsize=(8, 6))

for i in range(len(labels)):
    ax.scatter(depth_40[i], resolution[i], label=labels[i], s=200, marker='^')

# ax.set_xscale('log')
ax.set_yscale('log')
ax.set_ylim(ax.get_ylim()[::-1])  # Reverse the y-axis (range resolution)
ax.set_xlim(0, 1)

ax.set_xlabel('Depth (m)', fontsize=12)
ax.set_ylabel('Resolution (m)', fontsize=12)

ax.grid(True, which="both", ls="--", linewidth=0.5)
ax.legend(fontsize=9, loc='best')

plt.tight_layout()
plt.show()
