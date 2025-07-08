import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia import dataset


if __name__ == "__main__":

    dataset_dir = "../data/dry-bulk-density-dataset"
    dataset_raw = dataset.Dataset(dataset_dir)
