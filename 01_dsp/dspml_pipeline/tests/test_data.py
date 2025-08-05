import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dspml_pipeline.data.frame_loader import FrameLoader
from dspml_pipeline.setup_logging import setup_logging

if __name__ == "__main__":
    setup_logging(verbose=True)
    dataset_dir = "../data/wet-0-soil-compaction-dataset"
    frameLoader = FrameLoader(dataset_dir)
    frameLoader.read_frames()