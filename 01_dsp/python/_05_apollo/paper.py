# for plots from the paper

import matplotlib.pyplot as plt
import numpy as np

# data from https://www.nrcs.usda.gov/sites/default/files/2022-11/Bulk%20Density%20-%20Soil%20Health%20Guide_0.pdf
soil_textures = [
    "Sand,\nLoamy Sand",
    "Sandy Loam,\nLoam",
    "Sandy Clay Loam,\nClay Loam",
    "Silt,\nSilt Loam",
    "Silt Loam,\nSilty Clay Loam",
    "Sandy Clay,\nSilty Clay,\nClay Loam",
    "Clay\n(<45% clay)"
]

ideal = [1.60, 1.40, 1.40, 1.40, 1.40, 1.10, 1.10]
affects = [1.63, 1.63, 1.60, 1.60, 1.55, 1.49, 1.39]
restricts = [1.80, 1.80, 1.75, 1.75, 1.65, 1.58, 1.47]

x = np.arange(len(soil_textures))
width = 0.25

plt.figure(figsize=(12, 6))
plt.bar(x - width, ideal, width, label='Ideal for Plant Growth', color='skyblue')
plt.bar(x, affects, width, label='Affects Root Growth', color='orange')
plt.bar(x + width, restricts, width, label='Restricts Root Growth', color='crimson')

plt.ylabel('Bulk Density (g/cm³)')
plt.title('Bulk Density Thresholds for Root Growth by Soil Texture')
plt.xticks(x, soil_textures, rotation=45, ha='right')
plt.legend()
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()
