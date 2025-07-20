import numpy as np
import os
from pathlib import Path
import pandas as pd
import json
import pathlib
import sys
import shutil
from _06_hermes.parameters import num2label

class Dataset:
    """
    Class to handle the dataset for the GOPHER radar data.

    Parameters:
    dataset_dir (str):      The directory where the dataset is stored.
    new_dataset (bool):     If True, the class will load data from the dataset directory and save it as raw data. If False, it will load the raw data from saved files.
    ddc_flag (bool):        If True, the class will perform digital downconversion on the frames. Default is True.

    Attributes:
    dataset_dir (str):      The directory where the dataset is stored.
    X (np.ndarray):         The features of the dataset.
    y (np.ndarray):         The labels of the dataset.
    ddc (bool):             Flag to indicate if digital downconversion is performed.
    data_dir (Path):        The path to the dataset directory.
    data_log (Path):        The path to the data log CSV file.
    df (pd.DataFrame):      DataFrame containing the data log.

    """


    def __init__(self, dataset_dir):
        """
        Initialize the Dataset class.

        Args:
            dataset_dir (str): The directory where the dataset is stored.
        """

        self.data_dir = pathlib.Path(dataset_dir)
        if not self.data_dir.exists():
            print(f"Error: The directory {self.data_dir} does not exist.")
            sys.exit(1)
        self.data_log = self.data_dir / "data-log.csv"
        if not self.data_log.exists():
            print(f"Error: The data log {self.data_log} does not exist. ")
            sys.exit(1)
        self.df = pd.read_csv(self.data_log)
        self.df['Sample #'] = self.df['Sample #'].astype(str)
        self.df['Bulk Density (g/cm^3)'] = self.df['Bulk Density (g/cm^3)'].astype(float)
        self.full_monty()
    
    def label_dataset(self):
        """
        Create labels for the dataset by extracting bulk density from the data log.
        Each sample directory will have a JSON file with the bulk density.

        Args:
            None
        """

        for index, row in self.df.iterrows():
            sample_dir = self.data_dir / row["Sample #"]
            bulk_density = row["Bulk Density (g/cm^3)"]
            if sample_dir.exists():
                json_file = sample_dir / "data_params.json"
                if not json_file.exists():
                    with open(json_file, 'w') as f:
                        json.dump({"bulk-density": [bulk_density]}, f)
                else:
                    with open(json_file, 'r') as f:
                        data = f.read()
                    parsed = json.loads(data)
                    parsed["bulk-density"].append(bulk_density)
                    parsed["label"] = num2label(bulk_density)
                    with open(json_file, 'w') as f:
                        json.dump(parsed, f)
            else:
                print(f"Warning: {sample_dir} does not exist. Skipping.")
        
    def purge_labels(self):
        """
        Purge labels from the dataset for relabeling.
        Deletes the JSON files containing bulk density labels.

        Args:
            None
        """
        for index, row in self.df.iterrows():
            sample_dir = self.data_dir / row["Sample #"]
            json_file = sample_dir / "data_params.json"
            if json_file.exists():
                json_file.unlink()

    def frames_to_csv(self):
        """
        Converts all `.frames` files in subdirectories of the dataset directory into CSV files.
        Each resulting CSV contains the raw radar frame data.

        Args:
            None
        """
        root_path = self.data_dir
        subdirs = [f for f in root_path.iterdir() if f.is_dir() and f.name not in ['.', '..']]

        scans_per_datum = 1

        percentage = 0
        for folder in subdirs:
            frame_tot_combined = None
            frames_bb_combined = None
            capture_files = sorted(folder.glob('*.frames'))

            for j in range(0, len(capture_files), scans_per_datum):
                for i in range(j, min(j + scans_per_datum, len(capture_files))):
                    capture_file = capture_files[i]
                    frame_tot, params = process_frames(folder, capture_file.name)
                    
                    if frame_tot is None:
                        print(f"Skipping invalid capture: {capture_file}")
                        continue

                    if (i - j) == 0:
                        frame_tot_combined = frame_tot
                        width_frames = frame_tot.shape[1]
                    else:
                        frame_tot_combined = np.concatenate((frame_tot_combined, frame_tot), axis=1)

                # Save to CSV
                save_name = capture_file.stem + '_frameTot.csv'
                save_path = folder / save_name
                np.savetxt(save_path, frame_tot_combined, delimiter=',')

            # Save parameters to JSON, overwriting if it exists
            params_file = folder / "radar_params.json"
            with open(params_file, 'w') as f:
                json.dump(params, f, indent=4)
            percentage = percentage + (j + 1) / (len(capture_files) * len(subdirs)) * 100
            print(f"{percentage:.2f}%...")

    def full_monty(self):
        """
        Run the full preprocessing pipeline: label dataset, purge labels, and convert frames to CSV.
        
        Args:
            None
        """

        self.purge_labels()
        self.label_dataset()
        self.frames_to_csv()

def process_frames(file_path, capture_name):
    """
    Process Novelda radar data capture file to extract raw frames and baseband signal.

    Args:
        local_data_path (str):      Path to the data capture file.
        capture_name (str):         Name of the data capture file.

    Returns:
        frame_tot (np.ndarray):     Raw radar frames (shape: [num_samples, num_frames]).
        params (dict):              Dictionary containing potentially useful radar parameters.
    """

    if file_path is None or capture_name is None:
        print("Invalid local_data_path or capture_name!")
        return None, None

    file_path = file_path / capture_name

    with open(file_path, 'rb') as f:
        def fread(fmt, count=1):
            return np.fromfile(f, dtype=fmt, count=count)

        magic = fread(np.uint32)[0]
        FRAME_LOGGER_MAGIC_NUM = 0xFEFE00A2
        if magic != FRAME_LOGGER_MAGIC_NUM:
            print(f"Wrong data format: {capture_name}!")
            return None

        iterations = fread(np.int32)[0]
        pps = fread(np.int32)[0]
        dac_min = fread(np.int32)[0]
        dac_max = fread(np.int32)[0]
        dac_step = fread(np.int32)[0]

        radar_specifier = fread(np.int32)[0]

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

        num_samplers = fread(np.int32)[0]
        num_frames = fread(np.int32)[0]
        _ = fread(np.int32)[0]  # numRuns
        frame_rate = fread(np.int32)[0]
        _ = fread(np.float64, num_frames)  # times

        raw_data = fread(np.uint32, num_frames * num_samplers).astype(np.float64)
        raw_data = raw_data / (pps * iterations) * dac_step + dac_min
        frame_tot = raw_data.reshape((num_samplers, num_frames), order='F') # Each signal is a column. This was fun to debug :|
                                                                            # MATLAB stores matrices in column-major order by default.

        for i in range(num_frames):
            if np.max(frame_tot[:, i]) > 8191:
                if i > 0:
                    frame_tot[:, i] = frame_tot[:, i-1]
                else:
                    frame_tot[:, i] = frame_tot[:, i+1]

        _ = fread(np.float32)[0]  # fpsEst

        remaining = np.fromfile(f)
        if remaining.size > 0:
            print(f"FILE READ ERROR: {remaining.size} data remains! Check format match.")
            return None, None

    # Radar params (assumes you have this implemented)
    fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz = NoveldaChipParams(chip_set, pgen, '4mm')

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


    return frame_tot, params

def NoveldaChipParams(chip_set, pgen, sampler='4mm'):
    """
    Get Novelda radar chip parameters based on chipset, pulse generator (PGen), and sampler.

    Args:
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

# It would have been faster to rename the files in the dataset directories instead of writing this function.
def combine_datasets(dataset_dirs, target_dir):
    """
    Combine multiple datasets into a single dataset by renaming folders and copying files.
    Assumes that each directory is named "label-#-...". Eg.
    wet-0-soil-compaction-dataset, wet-1-soil-compaction-dataset.

    Args:
        dataset_dirs (list): List of dataset directories to combine.
        target_dir (str):    Directory where the combined dataset will be saved.
    """
    target_dir = pathlib.Path(target_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    
    for dataset_dir in dataset_dirs:
        print(f"Processing dataset: {dataset_dir}")

        cur_dataset = Dataset(dataset_dir)
        dataset_dir = cur_dataset.data_dir
        
        dataset_name = dataset_dir.name
        prefix = dataset_name.split("-")[0] + "-" + dataset_name.split("-")[1]  # "wet-0" or "wet-1"

        for folder in dataset_dir.iterdir():
            if folder.is_dir():
                new_folder_name = f"{prefix}-{folder.name}"
                target_folder = target_dir / new_folder_name
                target_folder.mkdir(exist_ok=True)
                for file in folder.iterdir():
                    if file.is_file():
                        target_file = target_folder / file.name
                        shutil.copy2(file, target_file)
        
        print(f"Dataset {dataset_dir} processed and copied to {target_dir}")