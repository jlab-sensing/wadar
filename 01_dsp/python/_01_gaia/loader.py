import pandas as pd
import glob
import os
import numpy as np
import json
import pathlib
from scipy import signal
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _02_poseidon import ddc

class FrameLoader:
    """
    FrameLoader is a class to load radar frames from a dataset directory. Each frame is expected to be in a CSV file, and the label is extracted from a JSON file in each folder.
    The class can also perform digital downconversion on the frames if specified. There is generally no reason to not perform DDC, as it is a standard step in radar signal processing.
    The class can save the raw data to .npy files for later use, and can also load the raw data from these files, or update those files with new data.

    Parameters:
    dataset_dir (str): The directory where the dataset is stored.
    new_dataset (bool): If True, the class will load data from the dataset directory and save it as raw data. If False, it will load the raw data from saved files.
    ddc (bool): If True, the class will perform digital downconversion on the frames. Default is True.
    """
    def __init__(self, dataset_dir, new_dataset=True, ddc_flag=True):
        self.dataset_dir = dataset_dir
        self.X = None
        self.y = None
        self.ddc = ddc_flag 
        if new_dataset:
            self.X, self.y = self.load_from_dataset()
            print("Beginning anomoly removal process. Estimated time: 1-2 minutes.")
            self.X = ddc.remove_anomalies(self.X) 
            if self.X is not None and self.y is not None:
                self.save_raw_data(self.X, self.y)
        else:
            self.X, self.y = self.load_raw_data()

    @staticmethod
    def extract_label(data_dir):
        data_dir = pathlib.Path(data_dir)
        json_file = data_dir / "data_params.json"
        if json_file.exists():
            with open(json_file, 'r') as f: 
                data = f.read()
            parsed = json.loads(data)
            label = parsed.get("bulk-density")   
            label = np.mean(label) if isinstance(label, list) else label
            return label
        else:
            print(f"Warning: {json_file} does not exist in {data_dir}. Returning default label 0.")
            return 0

    @staticmethod
    def load_frameTot(filename):
        frameTot = pd.read_csv(filename, header=None).values.astype('complex64')
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
        return X, y

    def load_from_dataset(self):
        all_X = []
        all_y = []
        for folder in os.listdir(self.dataset_dir):
            folder_path = os.path.join(self.dataset_dir, folder)
            if os.path.isdir(folder_path):
                X, y = self.load_from_file(folder_path)
                if self.ddc == True:
                    X = self.baseband_signal(X)
                if X is not None and y is not None:
                    all_X.append(X)
                    all_y.append(y)
                # print(f"Loaded {len(X)} frames from {folder_path} with label {y[0] if len(y) > 0 else 'N/A'}")
        if all_X and all_y:
            all_X = np.concatenate(all_X, axis=0)
            all_y = np.concatenate(all_y, axis=0)
            print("All data loaded successfully.")
            return all_X, all_y
        else:
            print("il n'y a pas de data")
            return None, None
    
    def baseband_signal(self, X):
        if X is None:
            return None
        for i in range(X.shape[0]):
            temp = X[i]
            for j in range(temp.shape[1]):
                temp[:, j] = ddc.novelda_digital_downconvert(temp[:, j])
            X[i] = temp
        return X

    def save_raw_data(self, X, y):
        np.save(os.path.join(self.dataset_dir, "X_raw.npy"), X)
        np.save(os.path.join(self.dataset_dir, "y_raw.npy"), y)
        print("Raw dataset saved as X_raw.npy and y_raw.npy")

    def load_raw_data(self):
        X = np.load(os.path.join(self.dataset_dir, "X_raw.npy"))
        y = np.load(os.path.join(self.dataset_dir, "y_raw.npy"))
        if X is None or y is None:
            raise ValueError("X_raw.npy and y_raw.npy not found in the dataset directory.")
        print("Loaded from existing dataset.")

        return X, y
    
    def update_dataset(self, X, y):
        if X.shape[0] != y.shape[0]:
            raise ValueError("Number of samples in X and y must match.")
        self.save_raw_data(X, y)

