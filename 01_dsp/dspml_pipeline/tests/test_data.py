import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dspml_pipeline.data.frame_loader import FrameLoader
from dspml_pipeline.setup_logging import setup_logging

if __name__ == "__main__":
    setup_logging(verbose=True)
    dataset_dirs = ["../data/wet-0-soil-compaction-dataset"]
    target_dir = "../data/training-dataset"
    frameLoader = FrameLoader(dataset_dirs, target_dir)
    frameLoader.read_frames()

    print(frameLoader.X.shape)
    print(frameLoader.Y.shape)