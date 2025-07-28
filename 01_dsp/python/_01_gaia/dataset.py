import numpy as np
from pathlib import Path
import pandas as pd
import json
import sys
import shutil
from typing import Optional, Tuple, Dict, Any
from _06_hermes.parameters import num2label

class Dataset:
    """
    Class to handle GOPHER radar dataset processing and labeling.

    This class processes raw .frames files, converts them to CSV format,
    and optionally applies labels from a data log file.

    Args:
        dataset_dir (str): Directory containing the dataset
        process_frames (bool): Whether to automatically process frames during initialization
        require_datalog (bool): Whether to require a data log file to exist
        verbose (bool): Whether to print detailed progress information

    Attributes:
        data_dir (Path): Path to the dataset directory
        data_log (Path): Path to the data log CSV file
        has_datalog (bool): Whether a data log file exists
        df (Optional[pd.DataFrame]): DataFrame containing the data log
        verbose (bool): Controls verbosity of output
    """

    def __init__(self, dataset_dir: str, process_frames: bool = True, 
                 require_datalog: bool = False, verbose: bool = True):
        """Initialize the Dataset class."""

        self.verbose = verbose
        self.data_dir = Path(dataset_dir)
        
        # Validate dataset directory
        if not self.data_dir.exists():
            self._error(f"Dataset directory does not exist: {self.data_dir}")
        
        # Check for data log
        self.data_log = self.data_dir / "data-log.csv"
        self.has_datalog = self.data_log.exists()
        
        if require_datalog and not self.has_datalog:
            self._error(f"Data log required but not found: {self.data_log}")
        
        # Load data log if available
        self.df = None
        if self.has_datalog:
            self._load_datalog()
        elif self.verbose:
            self._info(f"No data log found. Labeling will be skipped.")
        
        # Process frames if requested
        if process_frames:
            self.full_monty()

    def _load_datalog(self) -> None:
        """
        Load and validate the data log CSV file.
        """

        try:
            self.df = pd.read_csv(self.data_log)
            self.df['Sample #'] = self.df['Sample #'].astype(str)
            self.df['Bulk Density (g/cm^3)'] = self.df['Bulk Density (g/cm^3)'].astype(float)
            if self.verbose:
                self._info(f"Loaded data log with {len(self.df)} samples")
        except Exception as e:
            self._error(f"Failed to load data log: {e}")

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

    def label_dataset(self) -> None:
        """
        Create labels for the dataset by extracting bulk density from the data log.
        Each sample directory will have a JSON file with the bulk density.
        """

        if not self.has_datalog:
            self._warning("No data log available. Skipping labeling.")
            return

        self._info("Starting dataset labeling...")
        processed = 0
        skipped = 0

        for _, row in self.df.iterrows():
            sample_dir = self.data_dir / row["Sample #"]
            bulk_density = row["Bulk Density (g/cm^3)"]
            
            if not sample_dir.exists():
                self._warning(f"Sample directory does not exist: {sample_dir}")
                skipped += 1
                continue

            json_file = sample_dir / "data_params.json"
            label_data = {
                "bulk-density": [bulk_density],
                "label": num2label(bulk_density)
            }

            try:
                if json_file.exists():
                    with open(json_file, 'r') as f:
                        existing_data = json.load(f)
                    existing_data.update(label_data)
                    label_data = existing_data

                with open(json_file, 'w') as f:
                    json.dump(label_data, f, indent=2)
                processed += 1

            except Exception as e:
                self._warning(f"Failed to write label file {json_file}: {e}")
                skipped += 1

        self._info(f"Labeling complete: {processed} processed, {skipped} skipped")

    def purge_labels(self) -> None:
        """
        Remove all label files from the dataset for re-labeling.
        """

        if not self.has_datalog:
            self._warning("No data log available. Skipping label purging.")
            return
            
        self._info("Purging existing labels...")
        removed = 0
        
        for _, row in self.df.iterrows():
            sample_dir = self.data_dir / row["Sample #"]
            json_file = sample_dir / "data_params.json"
            
            if json_file.exists():
                try:
                    json_file.unlink()
                    removed += 1
                except Exception as e:
                    self._warning(f"Failed to remove label file {json_file}: {e}")
        
        self._info(f"Purged {removed} label files")

    def frames_to_csv(self) -> None:
        """
        Convert all .frames files in subdirectories to CSV format.
        Each resulting CSV contains the raw radar frame data.
        """

        self._info("Starting frame processing...")
        
        # Get all subdirectories
        all_items = list(self.data_dir.iterdir())
        
        subdirs = [d for d in self.data_dir.iterdir() 
                  if d.is_dir() and not d.name.startswith('.')]
                
        if not subdirs:
            self._warning("No subdirectories found for frame processing")
            # Debug: List what we actually found
            for item in all_items:
                self._info(f"  - {item.name} ({'directory' if item.is_dir() else 'file'})")
            return

        total_files = 0
        processed_files = 0
        failed_files = 0

        for i, folder in enumerate(subdirs, 1):
            self._info(f"Processing folder {i}/{len(subdirs)}: {folder.name}")
            
            capture_files = sorted(folder.glob('*.frames'))
            total_files += len(capture_files)
            
            self._info(f"Found {len(capture_files)} .frames files in {folder.name}")
            
            if not capture_files:
                self._warning(f"No .frames files found in {folder.name}")
                continue

            # Process each capture file
            params = None
            for capture_file in capture_files:
                try:
                    self._info(f"Processing: {capture_file.name}")
                    frame_data, params = process_frames(folder, capture_file.name)
                    
                    if frame_data is None:
                        self._warning(f"Failed to process: {capture_file.name}")
                        failed_files += 1
                        continue

                    # Save frame data to CSV
                    csv_path = folder / f"{capture_file.stem}_frameTot.csv"
                    np.savetxt(csv_path, frame_data, delimiter=',')
                    processed_files += 1
                    self._info(f"Saved CSV: {csv_path.name}")

                    if self.verbose and processed_files % 10 == 0:
                        progress = (processed_files / total_files) * 100
                        print(f"[INFO] Progress: {progress:.1f}% ({processed_files}/{total_files})")

                except Exception as e:
                    self._warning(f"Error processing {capture_file.name}: {e}")
                    failed_files += 1

            # Save radar parameters if we have any
            if params and len(capture_files) > 0:
                params_file = folder / "radar_params.json"
                try:
                    with open(params_file, 'w') as f:
                        json.dump(params, f, indent=2)
                    self._info(f"Saved parameters: {params_file.name}")
                except Exception as e:
                    self._warning(f"Failed to save parameters for {folder.name}: {e}")

        self._info(f"Frame processing complete: {processed_files} processed, {failed_files} failed")

    def full_monty(self) -> None:
        """
        Run the complete dataset processing pipeline.
        """

        self._info("Starting dataset processing pipeline...")
        
        if self.has_datalog:
            self.purge_labels()
            self.label_dataset()
        else:
            self._info("Skipping labeling steps (no data log available)")
        
        self.frames_to_csv()
        self._info("Dataset processing pipeline complete!")

    def process_frames_only(self) -> None:
        """
        Process frames to CSV without any labeling operations.
        Useful for datasets without data logs.
        """

        self._info("Processing frames only (no labeling)...")
        self.frames_to_csv()

def process_frames(file_path: Path, capture_name: str) -> Tuple[Optional[np.ndarray], Optional[Dict[str, Any]]]:
    """
    Process Novelda radar data capture file to extract raw frames.

    Parameters:
        file_path: Path to the directory containing the capture file
        capture_name: Name of the data capture file

    Returns:
        Tuple of (frame_data, parameters) or (None, None) if processing fails
        - frame_data: Raw radar frames (shape: [num_samples, num_frames])
        - Parameters: Dictionary containing radar parameters
    """

    if not file_path or not capture_name:
        print("[ERROR] Invalid file path or capture name")
        return None, None

    capture_path = file_path / capture_name

    if not capture_path.exists():
        print(f"[ERROR] Capture file does not exist: {capture_path}")
        return None, None

    try:
        with open(capture_path, 'rb') as f:
            def fread(fmt, count=1):
                return np.fromfile(f, dtype=fmt, count=count)

            # Read and validate header
            magic = fread(np.uint32)[0]
            FRAME_LOGGER_MAGIC_NUM = 0xFEFE00A2
            if magic != FRAME_LOGGER_MAGIC_NUM:
                print(f"[ERROR] Invalid data format in {capture_name}")
                return None, None

            # Read capture parameters
            iterations = fread(np.int32)[0]
            pps = fread(np.int32)[0]
            dac_min = fread(np.int32)[0]
            dac_max = fread(np.int32)[0]
            dac_step = fread(np.int32)[0]
            radar_specifier = fread(np.int32)[0]

            # Process radar-specific parameters
            if radar_specifier == 2:
                chip_set = "X2"
                samples_per_second = fread(np.float32)[0]
                pgen = fread(np.int32)[0]
                _ = fread(np.float32, 2)  # skip offsetDistance, sampleDelayToReference

            elif radar_specifier in (10, 11):
                chip_set = "X1-IPGO" if radar_specifier == 10 else "X1-IPG1"
                samples_per_second = fread(np.float64)[0]
                pgen = fread(np.int32)[0]
                _ = fread(np.int32, 2)  # skip samplingRate, clkDivider

            else:
                raise ValueError(f"Unknown radar specifier: {radar_specifier}")

            # Read frame data
            num_samplers = fread(np.int32)[0]
            num_frames = fread(np.int32)[0]
            _ = fread(np.int32)[0]  # numRuns
            frame_rate = fread(np.int32)[0]
            _ = fread(np.float64, num_frames)  # times

            # Process raw frame data
            raw_data = fread(np.uint32, num_frames * num_samplers).astype(np.float64)
            raw_data = raw_data / (pps * iterations) * dac_step + dac_min
            frame_data = raw_data.reshape((num_samplers, num_frames), order='F')

            # Clean up invalid frames
            for i in range(num_frames):
                if np.max(frame_data[:, i]) > 8191:
                    if i > 0:
                        frame_data[:, i] = frame_data[:, i-1]
                    elif i < num_frames - 1:
                        frame_data[:, i] = frame_data[:, i+1]

            _ = fread(np.float32)[0]  # fpsEst

            # Check for remaining data
            remaining = np.fromfile(f)
            if remaining.size > 0:
                print(f"[WARNING] {remaining.size} bytes of unread data in {capture_name}")

        # Get radar parameters
        fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz = NoveldaChipParams(chip_set, pgen, '4mm')

        # Create parameters dictionary
        params = {
            "iterations": int(iterations),
            "pps": int(pps),
            "dac_min": int(dac_min),
            "dac_max": int(dac_max),
            "dac_step": int(dac_step),
            "radar_specifier": int(radar_specifier),
            "chip_set": chip_set,
            "samples_per_second": float(samples_per_second),
            "pgen": int(pgen),
            "num_samplers": int(num_samplers),
            "num_frames": int(num_frames),
            "frame_rate": int(frame_rate),
            "fc": float(fc),
            "bw": float(bw),
            "bwr": float(bwr),
            "vp": float(vp),
            "n": int(n),
            "bw_hz": float(bw_hz),
            "pwr_dBm": float(pwr_dBm),
            "fs_hz": float(fs_hz)
        }

        return frame_data, params

    except Exception as e:
        print(f"[ERROR] Failed to process {capture_name}: {e}")
        return None, None

def NoveldaChipParams(chip_set, pgen, sampler='4mm'):
    """
    Get Novelda radar chip parameters based on chipset, pulse generator (PGen), and sampler.

    Parameters:
        chip_set (str):     Chipset type ('X1-IPG0', 'X1-IPG1', 'X2', 'X4').
        pgen (int):         Pulse generator index (0, 1, or 2 for X1, 0-11 for X2).
        sampler (str):      Sampler type ('4mm', '8mm', '4cm').

    Returns:
        fc (float):         Center frequency [Hz]
        bw (float):         Fractional bandwidth
        bwr (float):        dB down bandwidth range
        vp (float):         Peak voltage [V]
        n (int):            Number of samplers (frame size)
        bw_hz (float):      Bandwidth in Hz
        pwr_dBm (float):    Power in dBm
        fs_hz (float):      Sampling rate [Hz]
    """

    chip_set = chip_set.lower()
    sampler = sampler.lower()

    if chip_set == 'x1-ipg0':
        bwr = 12
        n = 512
        if pgen == 0:
            fL, fH, vp = 660e6, 7145e6, 0.47
        elif pgen == 1:
            fL, fH, vp = 845e6, 9550e6, 0.45
        elif pgen == 2:
            fL, fH, vp = 1060e6, 10410e6, 0.37
        else:
            raise ValueError("X1-IPG0 PGen must be 0, 1, or 2")
        fs_hz = {'4mm': 39e9, '8mm': 20e9, '4cm': 3.8e9}.get(sampler, 39e9)
        if sampler not in ['4mm', '8mm', '4cm']:
            print("Sampler must be '4mm', '8mm', or '4cm' for X1.")
        pwr_dBm = -19

    elif chip_set == 'x1-ipg1':
        bwr = 12
        n = 512
        vp = 0.5
        if pgen == 0:
            fL, fH = 435e6, 3165e6
        elif pgen == 1:
            fL, fH = 450e6, 3555e6
        elif pgen == 2:
            fL, fH = 485e6, 4065e6
        else:
            raise ValueError("X1-IPG1 PGen must be 0, 1, or 2")
        fs_hz = {'4mm': 39e9, '8mm': 20e9, '4cm': 3.8e9}.get(sampler, 39e9)
        if sampler not in ['4mm', '8mm', '4cm']:
            print("Sampler must be '4mm', '8mm', or '4cm' for X1.")
        pwr_dBm = -14

    elif chip_set == 'x2':
        bwr = 10
        n = 256
        fs_hz = 39e9
        vp_list = [0.69, 0.69, 0.72, 0.71, 0.72, 0.69, 0.65, 0.62, 0.62, 0.57, 0.54, 0.52]
        fc_list = [5.3e9, 5.4e9, 5.7e9, 6.1e9, 6.4e9, 6.8e9, 7.3e9, 7.7e9, 7.8e9, 8.2e9, 8.8e9, 9.1e9]
        bw_list = [1.75e9, 1.8e9, 1.85e9, 2.05e9, 2.15e9, 2.3e9, 2.35e9, 2.5e9, 2.5e9, 2.65e9, 3.1e9, 3.1e9]
        pwr_list = [-10.7, -10.8, -11.2, -11.8, -12.0, -12.6, -13.3, -14.0, -14.0, -14.8, -16.4, -16.7]

        if not (0 <= pgen < len(fc_list)):
            raise ValueError("X2 PGen must be in range 0 to 11")
        fc = fc_list[pgen]
        bw_hz = bw_list[pgen]
        vp = vp_list[pgen] / 2
        bw = bw_hz / fc
        pwr_dBm = pwr_list[pgen]

    elif chip_set == 'x4':
        bwr = 10
        n = 1536
        fs_hz = 23.328e9
        if pgen in [0, 3]:
            fc, bw_hz = 7.29e9, 1.4e9
        elif pgen in [1, 4]:
            fc, bw_hz = 8.748e9, 1.5e9
        else:
            raise ValueError("X4 PGen must be 0, 1, 3, or 4")
        bw = bw_hz / fc
        vp = 0.31
        pwr_dBm = -14

    else:
        raise ValueError("chipSet must be 'X1-IPG0', 'X1-IPG1', 'X2', or 'X4'")

    fc = (fH + fL) / 2 if 'fH' in locals() else fc
    bw = (fH - fL) / fc if 'fH' in locals() else bw
    bw_hz = fH - fL if 'fH' in locals() else bw_hz

    return fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz

def combine_datasets(dataset_dirs: list, target_dir: str, process_frames: bool = True, verbose: bool = True) -> None:
    """
    Combine multiple datasets into a single dataset by copying and renaming folders.
    
    Assumes each directory is named with pattern "label-#-...". 
    For example: wet-0-soil-compaction-dataset, wet-1-soil-compaction-dataset.

    Parameters:
        dataset_dirs: List of dataset directories to combine
        target_dir: Directory where the combined dataset will be saved
        verbose: Whether to print progress information
    """

    target_path = Path(target_dir)
    target_path.mkdir(parents=True, exist_ok=True)
    
    if verbose:
        print(f"[INFO] Combining {len(dataset_dirs)} datasets into {target_dir}")
    
    for i, dataset_dir in enumerate(dataset_dirs, 1):
        if verbose:
            print(f"[INFO] Processing dataset {i}/{len(dataset_dirs)}: {dataset_dir}")

        try:
            # Process dataset to ensure it's ready
            cur_dataset = Dataset(dataset_dir, process_frames=process_frames, verbose=verbose)
            dataset_path = cur_dataset.data_dir
            
            # Extract prefix from dataset name (e.g., "wet-0" from "wet-0-soil-compaction-dataset")
            dataset_name = dataset_path.name
            name_parts = dataset_name.split("-")
            if len(name_parts) >= 2:
                prefix = f"{name_parts[0]}-{name_parts[1]}"
            else:
                prefix = dataset_name

            folders_copied = 0
            files_copied = 0

            # Copy all subdirectories with new names
            for folder in dataset_path.iterdir():
                if not folder.is_dir():
                    continue
                    
                new_folder_name = f"{prefix}-{folder.name}"
                target_folder = target_path / new_folder_name
                target_folder.mkdir(exist_ok=True)
                
                # Copy all files from source to target
                for file in folder.iterdir():
                    if file.is_file():
                        target_file = target_folder / file.name
                        shutil.copy2(file, target_file)
                        files_copied += 1
                
                folders_copied += 1
            
            if verbose:
                print(f"[INFO] Copied {folders_copied} folders and {files_copied} files from {dataset_name}")
        
        except Exception as e:
            print(f"[ERROR] Failed to process dataset {dataset_dir}: {e}")
    
    if verbose:
        print(f"[INFO] Dataset combination complete! Combined dataset saved to {target_dir}")