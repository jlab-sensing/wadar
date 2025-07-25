import os
import sys
import numpy as np

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia import dataset
from _01_gaia.loader import FrameLoader
from _06_hermes.parameters import num2label


if __name__ == "__main__":

    dataset_dirs = ["../../data/wet-0-soil-compaction-dataset",
                    "../../data/wet-1-soil-compaction-dataset",
                    "../../data/wet-2-soil-compaction-dataset"]

    sample_matrix = np.zeros((3, len(dataset_dirs)))

    for i, dir in enumerate(dataset_dirs):
        dataset_raw = dataset.Dataset(dir)

        hydros = FrameLoader(dir, new_dataset=True, ddc_flag=True)
        X, y = hydros.X, hydros.y

        for j, bulk_density in enumerate(y):
            if num2label(bulk_density) == "Ideal":
                sample_matrix[0, i] += 1
            elif num2label(bulk_density) == "Restricted":
                sample_matrix[2, i] += 1
            else:
                sample_matrix[1, i] += 1

    print(sample_matrix)