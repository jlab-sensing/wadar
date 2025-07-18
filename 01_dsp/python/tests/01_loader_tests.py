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

    # This file assumes that the dataset directory is labeled.
    # See 01_dataset_tests.py for using the Dataset class
    # to label the directories properly.

    dataset_dir = "../data/combined-soil-compaction-dataset"

    # To load data from a dataset directory and save it as raw data
    hydros = FrameLoader(dataset_dir, new_dataset=True, ddc_flag=True)
    X, y = hydros.X, hydros.y

    # To load raw data from saved files
    # hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    # X, y = hydros.X, hydros.y

    print("X shape:", X.shape)
    print("y shape:", y.shape)

    plot_median_unique(X, y)