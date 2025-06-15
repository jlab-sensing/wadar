import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia import dataset
from _01_gaia import loader

if __name__ == "__main__":

    args = sys.argv[1:]
    if len(args) < 1:
        print("Usage: python PrepareDataset.py <dataset_dir> [new_dataset]")
        sys.exit(1)
    
    dataset_dir = args[0]
    dataset_raw = dataset.Dataset(dataset_dir)
    hydros = loader.FrameLoader(dataset_dir, new_dataset=True, ddc_flag=True)