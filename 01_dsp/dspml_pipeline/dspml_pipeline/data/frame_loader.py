import logging
logger = logging.getLogger(__name__)

import numpy as np
from pathlib import Path
from ..setup_logging import setup_logging
import json
import pandas as pd
import sys
from scipy import signal

THRESHOLD = 50 # For anomoly removal

class FrameLoader:
    """
    FrameLoader class for processing radar data into standardized input (X) and output (y) matrices for regression tasks.

    Expected directory structure for each dataset:
        dataset_dir/
            folder1/
                *.frames
            folder2/
                *.frames
            data_log.csv

    The data_log.csv file should contain:
        - A column with folder names (specified by folder_name)
        - A column with target labels (specified by label_name)

    Attributes:
        dataset_dirs (list):    List of dataset directory paths.
        target_dir (str):       Directory to save processed datasets.
        data_log (str):         Name of the data log CSV file (default: "data-log.csv").
        folder_name (str):      Column name in data_log for folder/sample names.
        label_name (str):       Column name in data_log for target labels.
        verbose (bool):         Verbosity flag for logging.
        X (np.ndarray):         Processed radar data (features).
        y (np.ndarray):         Corresponding labels (targets).
    """

    def __init__(self, dataset_dirs:list, target_dir:str,
                 data_log:str = "data-log.csv", 
                 folder_name:str = "Sampe $", label_name:str = "Bulk Density (g/cm^3)", 
                 verbose:bool = False):
        """
        Initializes the FrameLoader instance based on the provided directories.

        Args:
            dataset_dirs (list):    List of dataset directory paths.
            target_dir (str):       Directory to save processed datasets.
            data_log (str):         Name of the data log CSV file (default: "data-log.csv").
            folder_name (str):      Column name in data_log for folder/sample names.
            label_name (str):       Column name in data_log for target labels.
            verbose (bool):         Verbosity flag for logging.
        """
        self.verbose = verbose
        self.dataset_dirs = dataset_dirs
        self.target_dir = target_dir
        self.data_log = data_log
        self.X = None
        self.y = None
        self.label_name = label_name
        self.folder_name = folder_name

        # Validate dataset directory
        for i in self.dataset_dirs:
            if not Path(i).exists():
                logger.error(f"Dataset {i} does not exist.")
            data_log_i = Path(i) / data_log
            if not data_log_i.exists():
                logger.error(f"Data log file {data_log_i} does not exist.")
                sys.exit(1)

    def extract_data(self):
        """
        Extracts the features (X) and labels (y) from the provided directries.

        Returns:
            X (np.ndarray):         Processed radar data (features).
            y (np.ndarray):         Corresponding labels (targets).
        """

        logger.info("Starting frame processing")

        all_frame_data = []
        all_labels = []

        # Iterate through each dataset dir
        for i in self.dataset_dirs:
            dataset_dir = Path(i)
            subdirs = [d for d in dataset_dir.iterdir() 
                    if d.is_dir() and not d.name.startswith('.')]
            data_log = dataset_dir / self.data_log
            
            # Get the labels from the data log
            try:
                df = pd.read_csv(data_log)
                df[self.folder_name] = df[self.folder_name].astype(str)
                df[self.label_name] = df[self.label_name].astype(float)
                logger.info(f"Loaded data log with {len(df)} samples")
            except Exception as e:
                logger.error(f"Expected CSV format: columns include '{self.folder_name}' and '{self.label_name}'")
                sys.exit(1)
            
            # In each subdirectory
            for i, folder in enumerate(subdirs):

                capture_files = sorted(folder.glob("*.frames"))

                logger.info(f"Processing {len(capture_files)} files in {folder.name}")

                if not capture_files:
                    logger.warning(f"No .frames files found in {folder.name}")
                    continue

                # Find the row in df corresponding to this folder name
                sample_row = df[df['Sample #'] == folder.name]
                if sample_row.empty:
                    logger.error(f"No matching sample for folder {folder.name} in data log")
                    sys.exit(1)
                else:
                    bulk_density = sample_row.iloc[0][self.label_name]
                
                # Process each capture file
                params = None
                for capture_file in capture_files:
                    try:
                        frame_data, params = process_frames(folder, capture_file.name)

                        if frame_data is None:
                            logger.warning(f"Failed to process: {capture_file.name}")
                            continue

                        # Anomoly removal. Replaces values that deviate from the median by more
                        # than a threshold with the median. This has been done since the beginning 
                        # of the project because of odd spikes in the raw DAC output that causes
                        # large deviations in the data.
                        median = np.median(frame_data, axis=1, keepdims=True)
                        mask = np.abs(frame_data - median) > THRESHOLD
                        frame_data_clean = frame_data.copy()
                        frame_data_clean[mask] = np.broadcast_to(median, frame_data.shape)[mask]
                        
                        # DDC
                        ddc_frame_data = np.zeros_like(frame_data_clean, dtype=np.complex64)
                        for i in range(frame_data_clean.shape[1]):
                            ddc_frame_data[:, i] = novelda_digital_downconvert(frame_data_clean[:, i])
                        
                        try:
                            all_frame_data.append(ddc_frame_data)
                            all_labels.append(bulk_density)
                        except:
                            logger.error(f"Failed to stack radar data from {capture_file.name}")
                            sys.exit(1)

                    # Outputs warning when problem occurs while processing, but continues processing other radar data.
                    except Exception as e:
                        logger.warning(f"Error processing {capture_file.name}: {e}")

            # Save radar parameters
            if params and len(capture_files) > 0:
                params_file = folder / "radar_params.json"
                with open(params_file, 'w') as f:
                    json.dump(params, f)
                logger.info(f"Saved parameters: {params_file.name}")

        self.X = np.stack(all_frame_data)
        self.y = np.stack(all_labels)

        return self.X, self.y
    
    def save_dataset(self):
        """
        Saves dataset for future processing. Data is automatically stored as X.npy and y.npy.
        """

        if not Path(self.target_dir).exists():
            Path(self.target_dir).mkdir(parents=True)

        X_path = Path(self.target_dir) / "X.npy"
        y_path = Path(self.target_dir) / "y.npy"

        np.save(X_path, self.X)
        np.save(y_path, self.y)

        logger.info(f"Raw dataset saved as X.npy and y.npy")
        logger.info(f"Saved shapes: X={self.X.shape}, y={self.y.shape}")

def load_dataset(dataset_dir:str):
    """
    Loads data that has already been processed. Assumes the features are named X.npy and the 
    labels are named y.npy.

    Args:
        dataset_dir:        Directory containing the capture file.

    Returns:
        X (np.ndarray):     Processed radar data (features).
        y (np.ndarray):     Corresponding labels (targets).
    """

    X_path = Path(dataset_dir) / "X.npy"
    y_path = Path(dataset_dir) / "y.npy"

    if not X_path.exists() or not y_path.exists():
        logger.error("X.npy and/or y.npy not found in the dataset directory")
        sys.exit(1)

    X = np.load(X_path)
    y = np.load(y_path)
    
    logger.info(f"Loaded from existing dataset: X={X.shape}, y={y.shape}")

    return X, y
        
def process_frames(file_path:Path, capture_name:str):
    """
    Process Novelda radar data capture file to extract raw frames.

    Args:
        file_path (Path):           Path to the directory containing the capture file.
        capture_name (str):         Name of the data capture file

    Returns:
        frame_data (np.ndarray):    Raw radar frames (slow time, fast time)
        Parameters (dict):          Dictionary containing radar parameters
    """

    if not file_path or not capture_name:
        logger.error("Invalid file path or capture name")
        sys.exit(1)
        return None, None

    capture_path = file_path / capture_name

    if not capture_path.exists():
        logger.error(f"Capture file does not exist: {capture_path}")
        sys.exit(1)
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
                logger.error(f"Unknown radar specifier: {radar_specifier}")
                sys.exit(1)

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
                logger.error(f"{remaining.size} bytes of unread data in {capture_name}")
                sys.exit(1)

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
        logger.errir(f"Failed to process {capture_name}: {e}")
        return None, None
    
def NoveldaChipParams(chip_set:str, pgen:int, sampler:str='4mm'):
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

    # It will always run this for the Chipotle radar. Every other option exists because
    # the original NoveldaChipParams.m provided by FlatEarth included them.
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

def novelda_digital_downconvert(raw_frame:np.ndarray):
    """
    Function to apply a digital downcovert (DDC) to a high frequency radar
    signal. Brings signal to baseband frequencies and provides an analytic
    signal (i.e. I & Q, in-phase & quadrature, outputs). Inherited from
    NoveldaDDC.m.

    TODO: Quite certain this can be optimized, but since it's derived from
    DSP concepts I do not fully understand, I am leaving it as is for now.
    If you see this comment and I have already graduated, maybe message me
    because I am quite curious whether I'd be able to implement a better
    matched filter.

    Args:
        raw_frame (np.ndarray):         Input radar frame data, shape (N,).

    Returns:
        baseband_signal (np.ndarray):   Baseband signal after DDC, shape (N,).
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