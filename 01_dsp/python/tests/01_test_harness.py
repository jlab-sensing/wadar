
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader


if __name__ == "__main__":

    # To load data from a dataset directory and save it as raw data
    dataset_dir = "../data/compact-4-dry"
    hydros = FrameLoader(dataset_dir, new_dataset=True, ddc=True)
    X, y = hydros.X, hydros.y

    # To load raw data from saved files
    dataset_dir = "../data/compact-4-dry"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc=True)
    X, y = hydros.X, hydros.y