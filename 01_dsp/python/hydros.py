# Hydros (Ὕδρος) is the Greek primordial deity of water, representing the vast and life-giving element that sustains all forms of life. Here
# it represnets the flow of raw, undifferentiated data, ready to be processed and transformed into meaningful insights.

import pandas as pd
import glob
import os
import numpy as np
import json
import pathlib
from scipy import signal

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
        print(f"Loaded {len(X)} samples with shape {X.shape} and labels shape {y.shape}")
        return X, y

    def load_from_dataset(self):
        all_X = []
        all_y = []
        for folder in os.listdir(self.dataset_dir):
            folder_path = os.path.join(self.dataset_dir, folder)
            if os.path.isdir(folder_path):
                X, y = self.load_from_file(folder_path)
                X = self.baseband_signal(X)
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
    
    def baseband_signal(self, X):
        if X is None:
            return None
        for i in range(X.shape[0]):
            temp = X[i]
            for j in range(temp.shape[1]):
                temp[:, j] = novelda_digital_downconvert(temp[:, j])
            X[i] = temp
        return X

    def save_raw_data(self, X, y):
        np.save(os.path.join(self.dataset_dir, "X_raw.npy"), X)
        np.save(os.path.join(self.dataset_dir, "y_raw.npy"), y)
        print("Raw dataset saved as X_raw.npy and y_raw.npy")

    def load_raw_data(self):
        X = np.load(os.path.join(self.dataset_dir, "X_raw.npy"))
        y = np.load(os.path.join(self.dataset_dir, "y_raw.npy"))
        print(f"Loaded raw dataset with shape {X.shape} and labels shape {y.shape}")
        return X, y
    
    def update_dataset(self, X, y):
        if X.shape[0] != y.shape[0]:
            raise ValueError("Number of samples in X and y must match.")
        self.save_raw_data(X, y)

# Private function because it is not intended to be used outside this module.
def novelda_digital_downconvert(raw_frame):
    """
    Function to apply a digital downcovert (DDC) to a high frequency radar
    signal. Brings signal to baseband frequencies and provides an analytic
    signal (i.e. I & Q, in-phase & quadrature, outputs). Inherited from
    NoveldaDDC.m.

    :param raw_frame:   high-frequency sampled UWB radar signal
    :return:            baseband, and filtered IQ radar 
                        signal, note: use abs() on output to get envelope
                        magnitude
    """

    # These parameters are for true the X1-IPG1
    Fs = 39e9           # Mean system sampling rate for 4mm
    fL = 435e6;         # -10 dB low cutoff for pgen 0
    fH = 3165e6;        # -10 dB high cutoff
    Fc = (fH + fL) / 2

    # Digital Down-Convert parameters (normalized frequency index)
    N = len(raw_frame)                             
    freqIndex = Fc / Fs * N
    t = np.linspace(0, 1, N, endpoint=False)

    # Generate the complex sinusoid LO (local oscillator) to mix into the signal 
    phi = 0         # Phase offset?
    LO = np.sin(2 * np.pi * freqIndex * t + phi) + 1j * np.cos(2 * np.pi * freqIndex * t + phi)

    # Digital Downconvert (the DDC) via direct multiplication subtracting the mean removes DC offset
    rf_signal = raw_frame - np.mean(raw_frame)
    mixed = rf_signal * LO

    # LPF Design to eliminate the upper mixing frequencies after the DDC (21- tap hamming window)
    M = 20                                  # Filter order, equal to # of filter taps - 1. Inherited this from NoveldaDDC.m.
    window = np.hamming(M + 1)
    window /= np.sum(window[:(M//2 + 1)])   # Normalize the weights

    # Baseband signal using convolution (provides downcoverted, filtered analytic signal)
    baseband_signal = signal.convolve(mixed, window, mode='same')

    return baseband_signal

# ===========================
# Test Harness
# ===========================
if __name__ == "__main__":

    # Example usage:

    # To load data from a dataset directory and save it as raw data
    dataset_dir = "data/compact-4-dry"
    hydros = HydrosFrameLoader(dataset_dir)
    X, y = hydros.load_from_dataset()
    if X is not None and y is not None:
        hydros.save_raw_data(X, y)

    # To load raw data from saved files
    # dataset_dir = "data/compact-4-dry"
    # hydros = HydrosFrameLoader(dataset_dir)
    # X, y = hydros.load_raw_data()