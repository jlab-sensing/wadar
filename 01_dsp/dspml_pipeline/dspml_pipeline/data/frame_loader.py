import logging
logger = logging.getLogger(__name__)

import numpy as np
from pathlib import Path
from ..setup_logging import setup_logging


class FrameLoader:
    def __init__(self, dataset_dir: str, data_log: str = "data-log.csv", verbose: bool = False):
        self.verbose = verbose
        self.dataset_dir = Path(dataset_dir)
        self.data_log = self.dataset_dir / data_log

        # Validate dataset directory
        if not self.dataset_dir.exists():
            raise FileNotFoundError(f"Dataset {dataset_dir} does not exist.")
        
        # Check for datalog
        if not self.data_log.exists():
            raise FileNotFoundError(f"Data log file {self.data_log} does not exist.")

    def read_frames(self) -> np.array:
        logger.info("Starting frame processing")

        subdirs = [d for d in self.dataset_dir.iterdir() 
                  if d.is_dir() and not d.name.startswith('.')]
        
        for i, folder in enumerate(subdirs):
            logger.info(f"Processing folder {i}/{len(subdirs)}: {folder.name}")

            capture_files = sorted(folder.glob("*.frames"))

            logger.info(f"Processing {len(capture_files)} files in {folder.name}")

            if not capture_files:
                logger.warning(f"No .frames files found in {folder.name}")
                continue

            params = None
            for capture_file in capture_files:
                logger.info(f"Processing: {capture_file.name}")