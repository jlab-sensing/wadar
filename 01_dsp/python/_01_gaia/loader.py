import pandas as pd
import glob
import os
import numpy as np
import json
from pathlib import Path
from scipy import signal
import sys
from typing import Optional, Tuple

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
sys.path.insert(0, parent_dir)
from _02_poseidon import ddc


class FrameLoader:
    """
    FrameLoader loads radar frames from a dataset directory and processes them.
    
    Each frame is expected to be in a CSV file, with labels extracted from JSON files 
    in each folder. The class can perform digital downconversion and anomaly removal.

    Args:
        dataset_dir (str): Directory containing the dataset
        new_dataset (bool): If True, load from CSV files; if False, load from saved .npy files
        ddc_flag (bool): Whether to perform digital downconversion
        verbose (bool): Whether to print detailed progress information
    
    Attributes:
        dataset_dir (str): Path to the dataset directory
        X (np.ndarray): Loaded feature data
        y (np.ndarray): Loaded label data
        ddc (bool): Digital downconversion flag
        verbose (bool): Verbosity control
    """
    
    def __init__(self, dataset_dir: str, new_dataset: bool = True, 
                 ddc_flag: bool = True, verbose: bool = True):
        """
        Initialize the FrameLoader.
        """
        self.dataset_dir = dataset_dir
        self.verbose = verbose
        self.X = None
        self.y = None
        self.ddc = ddc_flag
        
        # Validate dataset directory
        if not os.path.exists(dataset_dir):
            self._error(f"Dataset directory does not exist: {dataset_dir}")
        
        if new_dataset:
            self._load_and_process_new_dataset()
        else:
            self._load_existing_dataset()

    def _info(self, message: str) -> None:
        """
        Print info message if verbose mode is enabled.
        """
        if self.verbose:
            print(f"[INFO] {message}")

    def _warning(self, message: str) -> None:
        """
        Print warning message.
        """
        
        print(f"[WARNING] {message}")

    def _error(self, message: str) -> None:
        """
        Print error message and exit.
        """

        print(f"[ERROR] {message}")
        sys.exit(1)

    def _load_and_process_new_dataset(self) -> None:
        """
        Load dataset from CSV files and process it.
        """

        self._info("Loading dataset from CSV files...")
        self.X, self.y = self.load_from_dataset()
        
        if self.X is None or self.y is None:
            self._error("Failed to load dataset from CSV files")
        
        self._info(f"Loaded {self.X.shape[0]} samples with {self.X.shape[1:]} frame shape")
        
        # Remove anomalies
        if self.X is not None:
            self._info("Beginning anomaly removal process...")
            original_shape = self.X.shape
            self.X = ddc.remove_anomalies(self.X)
            if self.X is not None:
                self._info(f"Anomaly removal complete. Shape: {original_shape} -> {self.X.shape}")
            else:
                self._warning("Anomaly removal returned None")
        
        # Save processed data
        if self.X is not None and self.y is not None:
            self.save_raw_data(self.X, self.y)

    def _load_existing_dataset(self) -> None:
        """
        Load dataset from existing .npy files.
        """
        self._info("Loading dataset from existing .npy files...")
        self.X, self.y = self.load_raw_data()
        if self.X is not None and self.y is not None:
            self._info(f"Loaded {self.X.shape[0]} samples from saved files")

    @staticmethod
    def extract_label(data_dir: str) -> float:
        """
        Extract label from data_params.json file in the given directory.
        
        Parameters:
            data_dir: Path to the directory containing data_params.json
            
        Returns:
            Extracted label value or 0.0 if not found
        """

        data_path = Path(data_dir)
        json_file = data_path / "data_params.json"
        
        if not json_file.exists():
            print(f"[WARNING] {json_file} does not exist in {data_dir}. Returning default label 0.")
            return 0.0
            
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            label = data.get("bulk-density")
            if label is None:
                print(f"[WARNING] No 'bulk-density' key found in {json_file}")
                return 0.0
                
            # Handle list or single value
            if isinstance(label, list):
                return float(np.mean(label))
            else:
                return float(label)
                
        except Exception as e:
            print(f"[ERROR] Failed to read label from {json_file}: {e}")
            return 0.0

    @staticmethod
    def load_frameTot(filename: str) -> Optional[np.ndarray]:
        """
        Load frame data from CSV file.
        
        Parameters:
            filename: Path to the CSV file
            
        Returns:
            Frame data as complex64 array or None if loading fails
        """

        try:
            frameTot = pd.read_csv(filename, header=None).values.astype('complex64')
            return frameTot
        except Exception as e:
            print(f"[ERROR] Failed to load frame data from {filename}: {e}")
            return None

    def load_from_file(self, csv_dir: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Load all CSV files from a directory and extract labels.
        
        Parameters:
            csv_dir: Directory containing CSV files and data_params.json
            
        Returns:
            Tuple of (features, labels) or (None, None) if loading fails
        """

        # Extract label for this directory
        label = self.extract_label(csv_dir)
        
        if label == 0.0:
            if self.verbose:
                self._warning(f"Skipping folder with no valid label: {csv_dir}")
            return None, None
        
        # Find all CSV files
        csv_files = glob.glob(os.path.join(csv_dir, '*.csv'))
        
        if not csv_files:
            self._warning(f"No CSV files found in {csv_dir}")
            return None, None
        
        if self.verbose:
            self._info(f"Loading {len(csv_files)} CSV files from {os.path.basename(csv_dir)}")
        
        all_features = []
        all_labels = []
        
        for file in csv_files:
            features = self.load_frameTot(file)
            if features is not None:
                all_features.append(features)
                all_labels.append(label)
            else:
                self._warning(f"Failed to load {file}")
        
        if not all_features:
            self._warning(f"No valid features loaded from {csv_dir}")
            return None, None
        
        try:
            X = np.stack(all_features)
            y = np.array(all_labels, dtype='float32')
            return X, y
        except Exception as e:
            self._error(f"Failed to stack features from {csv_dir}: {e}")
            return None, None

    def load_from_dataset(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Load the entire dataset from all subdirectories.
        
        Returns:
            Tuple of (all_features, all_labels) or (None, None) if loading fails
        """

        self._info(f"Scanning dataset directory: {self.dataset_dir}")
        
        # Get all subdirectories
        folders = [f for f in os.listdir(self.dataset_dir) 
                  if os.path.isdir(os.path.join(self.dataset_dir, f))]
        
        if not folders:
            self._error(f"No subdirectories found in {self.dataset_dir}")
            return None, None
        
        self._info(f"Found {len(folders)} subdirectories to process")
        
        all_X = []
        all_y = []
        processed_folders = 0
        skipped_folders = 0
        
        for folder in folders:
            folder_path = os.path.join(self.dataset_dir, folder)
            
            # Load data from this folder
            X, y = self.load_from_file(folder_path)
            
            # Apply DDC if requested
            if X is not None and self.ddc:
                if self.verbose:
                    self._info(f"Applying DDC to {folder}")
                X = self.baseband_signal(X)
            
            # Add to collection if successful
            if X is not None and y is not None:
                all_X.append(X)
                all_y.append(y)
                processed_folders += 1
                
                if self.verbose:
                    self._info(f"Loaded {len(X)} frames from {folder} with label {y[0]:.3f}")
            else:
                skipped_folders += 1
        
        self._info(f"Processing complete: {processed_folders} folders processed, {skipped_folders} skipped")
        
        if not all_X:
            self._error("No valid data loaded from any folder")
            return None, None
        
        try:
            # Concatenate all data
            all_X = np.concatenate(all_X, axis=0)
            all_y = np.concatenate(all_y, axis=0)
            
            self._info(f"Successfully loaded {all_X.shape[0]} total samples")
            return all_X, all_y
            
        except Exception as e:
            self._error(f"Failed to concatenate data: {e}")
            return None, None
    
    def baseband_signal(self, X: np.ndarray) -> Optional[np.ndarray]:
        """
        Apply digital downconversion to the signal data.

        Parameters:
            X: Input signal data
            
        Returns:
            Processed signal data or None if processing fails
        """

        if X is None:
            return None
            
        try:
            if self.verbose:
                self._info(f"Applying DDC to {X.shape[0]} frames...")
                
            for i in range(X.shape[0]):
                temp = X[i]
                for j in range(temp.shape[1]):
                    temp[:, j] = ddc.novelda_digital_downconvert(temp[:, j])
                X[i] = temp
                
                # Progress update for large datasets
                if self.verbose and i > 0 and i % 50 == 0:
                    progress = (i / X.shape[0]) * 100
                    print(f"[INFO] DDC Progress: {progress:.1f}% ({i}/{X.shape[0]})")
            
            if self.verbose:
                self._info("DDC processing complete")
            return X
            
        except Exception as e:
            self._error(f"DDC processing failed: {e}")
            return None

    def save_raw_data(self, X: np.ndarray, y: np.ndarray) -> None:
        """
        Save processed data to .npy files.
        
        Parameters:
            X: Feature data to save
            y: Label data to save
        """

        try:
            x_path = os.path.join(self.dataset_dir, "X_raw.npy")
            y_path = os.path.join(self.dataset_dir, "y_raw.npy")
            
            np.save(x_path, X)
            np.save(y_path, y)
            
            self._info(f"Raw dataset saved as X_raw.npy and y_raw.npy")
            self._info(f"Saved shapes: X={X.shape}, y={y.shape}")
            
        except Exception as e:
            self._error(f"Failed to save raw data: {e}")

    def load_raw_data(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Load data from existing .npy files.
        
        Returns:
            Tuple of (X, y) data or (None, None) if loading fails
        """

        try:
            x_path = os.path.join(self.dataset_dir, "X_raw.npy")
            y_path = os.path.join(self.dataset_dir, "y_raw.npy")
            
            if not os.path.exists(x_path) or not os.path.exists(y_path):
                self._error("X_raw.npy and/or y_raw.npy not found in the dataset directory")
                return None, None
            
            X = np.load(x_path)
            y = np.load(y_path)
            
            if X is None or y is None:
                self._error("Loaded data is None")
                return None, None
            
            self._info(f"Loaded from existing dataset: X={X.shape}, y={y.shape}")
            return X, y
            
        except Exception as e:
            self._error(f"Failed to load raw data: {e}")
            return None, None
    
    def update_dataset(self, X: np.ndarray, y: np.ndarray) -> None:
        """
        Update the saved dataset with new data.
        
        Parameters:
            X: New feature data
            y: New label data
        """

        if X.shape[0] != y.shape[0]:
            raise ValueError("Number of samples in X and y must match")
        
        self.save_raw_data(X, y)
        self.X = X
        self.y = y
        
        if self.verbose:
            self._info(f"Dataset updated with {X.shape[0]} samples")

