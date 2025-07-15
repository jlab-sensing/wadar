import os
import sys
import numpy as np
import matplotlib.pyplot as plt

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _05_apollo.viz_tools import plot_median_unique
from _01_gaia import dataset


if __name__ == "__main__":

    dataset_dir = "../data/dry-soil-compaction-dataset"
    
    # To load data from a dataset directory and save it as raw data
    dataset_raw = dataset.Dataset(dataset_dir)
    hydros = FrameLoader(dataset_dir, new_dataset=True, ddc_flag=True)
    X, y = hydros.X, hydros.y

    # To load raw data from saved files
    # hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    # X, y = hydros.X, hydros.y

    print("X shape:", X.shape)

    # plt.figure(figsize=(6, 6))
    # signal = np.abs(X[1, :, 100])
    # plt.plot(signal, color='black', linewidth=1)
    # # plt.grid(False)
    # # plt.xticks([])
    # # plt.yticks([])
    # plt.tight_layout()
    # plt.box(False)
    # plt.show()

    plot_median_unique(X, y)