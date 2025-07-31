import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _05_apollo import viz_tools

# A bunch of figures to visually see the difference between soil compaction 
# levels. If you are looking into how to use the functions in this
# pipeline, this is not the file to look at.

if __name__ == "__main__":

    # load the dataset
    dataset_dir = "../data/combined-training-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    viz_tools.plot_IQ_signals(X, y)
    plt.show()