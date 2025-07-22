import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia import dataset


if __name__ == "__main__":

    # If combining datasets
    dataset_dirs = ["../data/wet-0-soil-compaction-dataset",
                    "../data/wet-1-soil-compaction-dataset",
                    "../data/wet-2-soil-compaction-dataset"]
    
    target_dir = "../data/combined-soil-compaction-dataset"

    dataset.combine_datasets(dataset_dirs, target_dir)

    # # If using a single dataset
    # dataset_dir = "../data/wet-0-soil-compaction-dataset"
    # dataset_raw = dataset.Dataset(dataset_dir)