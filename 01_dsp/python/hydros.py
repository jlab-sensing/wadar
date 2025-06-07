# Hydros (Ὕδρος) is the Greek primordial deity of water, representing the vast and life-giving element that sustains all forms of life. Here
# it represnets the flow of raw, undifferentiated data, ready to be processed and transformed into meaningful insights.

import tensorflow as tf
import pandas as pd
import glob
import os
import numpy as np
import json
import pathlib

class HydrosFrameLoader:
    def __init__(self, dataset_dir):
        self.dataset_dir = dataset_dir

    @staticmethod
    def extract_label(data_dir):
        data_dir = pathlib.Path(data_dir)
        json_file = data_dir / "data_params.json"
        if json_file.exists():
            with open(json_file, 'r') as f: 
                data = f.read()
            parsed = json.loads(data)
            label = parsed.get("bulk-density")   
            label = np.median(label) if isinstance(label, list) else label
            return label
        else:
            print(f"Warning: {json_file} does not exist in {data_dir}. Returning default label 0.")
            return 0

    @staticmethod
    def load_frameTot(filename):
        frameTot = pd.read_csv(filename, header=None).values.astype('float32')
        return frameTot

    def load_from_file(self, csv_dir):
        all_features = []
        all_labels = []
        label = self.extract_label(csv_dir)
        csv_files = glob.glob(os.path.join(csv_dir, '*.csv'))
        if label == 0:
            print("Skipping folder with no label:", csv_dir)
            return None, None
        for file in csv_files:
            features = self.load_frameTot(file)
            all_features.append(features)
            all_labels.append(label)
        X = np.stack(all_features)
        y = np.array(all_labels, dtype='float32')
        print(f"Loaded {len(X)} samples with shape {X.shape} and labels shape {y.shape}")
        return X, y

    def load_from_dataset(self):
        all_X = []
        all_y = []
        for folder in os.listdir(self.dataset_dir):
            folder_path = os.path.join(self.dataset_dir, folder)
            if os.path.isdir(folder_path):
                X, y = self.load_from_file(folder_path)
                if X is not None and y is not None:
                    all_X.append(X)
                    all_y.append(y)
        if all_X and all_y:
            all_X = np.concatenate(all_X, axis=0)
            all_y = np.concatenate(all_y, axis=0)
            print(f"Total samples loaded: {len(all_X)} with shape {all_X.shape} and labels shape {all_y.shape}")
            return all_X, all_y
        else:
            print("il n'y a pas de data")
            return None, None

    def save_raw_data(self, X, y):
        np.save(os.path.join(self.dataset_dir, "X_raw.npy"), X)
        np.save(os.path.join(self.dataset_dir, "y_raw.npy"), y)
        print("Raw dataset saved as X_raw.npy and y_raw.npy")

    def load_raw_data(self):
        X = np.load(os.path.join(self.dataset_dir, "X_raw.npy"))
        y = np.load(os.path.join(self.dataset_dir, "y_raw.npy"))
        print(f"Loaded raw dataset with shape {X.shape} and labels shape {y.shape}")
        return X, y

# ===========================
# Test Harness
# ===========================
if __name__ == "__main__":

    # Example usage:

    # To load data from a dataset directory and save it as raw data
    # dataset_dir = "data/compact-4-dry"
    # hydros = HydrosFrameLoader(dataset_dir)
    # X, y = hydros.load_from_dataset()
    # if X is not None and y is not None:
    #     hydros.save_raw_data(X, y)

    # To load raw data from saved files
    dataset_dir = "data/compact-4-dry"
    hydros_loader = HydrosFrameLoader(dataset_dir)
    X, y = hydros_loader.load_raw_data()