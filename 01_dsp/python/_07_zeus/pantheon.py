# Pantheon: End-to-End Evaluation Script

import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _01_gaia.dataset import Dataset, combine_datasets

if __name__ == "__main__":

    raw_training_datasets = [
        "../data/wet-0-soil-compaction-dataset",
        "../data/wet-1-soil-compaction-dataset",
        "../data/wet-2-soil-compaction-dataset"]
    target_training_datasets = "../data/training-dataset"
    raw_validation_datasets = [
        "../data/field-soil-compaction-dataset"]
    target_validation_dataset = "../data/validation-dataset"

    new_dataset = False

    # ====================================================

    training_dataset = target_training_datasets
    validation_dataset = target_validation_dataset

    if new_dataset:
        combine_datasets(raw_training_datasets, target_training_datasets, verbose=True)
        combine_datasets(raw_validation_datasets, target_validation_dataset, verbose=True)
    
    training_frame_loader = FrameLoader(training_dataset, new_dataset=new_dataset, ddc_flag=True)
    X_train, y_train = training_frame_loader.X, training_frame_loader.y

    validation_frame_loader = FrameLoader(validation_dataset, new_dataset=new_dataset, ddc_flag=True)
    X_val, y_val = validation_frame_loader.X, validation_frame_loader.y