"""
Handcrafted feature engineering tools for radar signal processing. 
"""

import logging
logger = logging.getLogger(__name__)

import numpy as np
import pandas as pd
from scipy.signal import find_peaks, peak_widths
from scipy.stats import skew, kurtosis

def get_median_frames(X:np.ndarray):
    """
    Get the median frame across slow time.

    Args:
        X (np.ndarray):             Raw data (samples, fast time, slow time)

    Returns:
        median_frames (np.ndarray): Median frames of shape (samples, fast_time)
    """

    logger.info("Computing median frame across slow time")

    median_frames = np.median(X, axis=2)

    return median_frames

def get_peak_idx(median_frames:np.ndarray):
    """
    Computes the two main peaks across all frames.

    Args:
        median_frames (np.ndarray): Median frames of shape (samples, fast_time)

    Returns:
        peak_idxs (np.ndarray):     Peak indices of shape (samples, 2)
    """

    logger.info("Computing peak indices for all samples")

    peak_idxs = np.zeros((median_frames.shape[0], 2), dtype=int)
    
    for i in range(median_frames.shape[0]):
        signal = np.abs(median_frames[i])
        
        # Find peaks with minimum distance and height constraints
        peaks, _ = find_peaks(signal, distance=5, height=np.max(signal) * 0.05)
        
        if len(peaks) == 0:
            # Fallback: use max and a reasonable second peak
            max_idx = np.argmax(signal)
            second_idx = max_idx // 2 if max_idx > 1 else min(max_idx + 1, len(signal) - 1)
            peak_idxs[i] = [max_idx, second_idx]
        elif len(peaks) == 1:
            # Only one peak found, duplicate it
            peak_idxs[i] = [peaks[0], peaks[0]]
        else:
            # Get indices of the two largest peaks
            top2_indices = np.argsort(signal[peaks])[-2:]
            peak_idxs[i] = peaks[top2_indices]
        
    return peak_idxs

def get_peak_amplitude(median_frames: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract amplitude of the two largest peaks.

    Args:
        median_frames (np.ndarray): Median frames of shape (samples, fast_time)
        peak_idxs (np.ndarray):     Peak indices of shape (samples, 2)

    Returns:
        peak_amplitudes (np.ndarray): Peak amplitudes of shape (samples, 2)
    """
    
    logger.info("Extracting peak amplitudes...")
    
    signal = np.abs(median_frames)
    
    # Peak amplitude extraction
    samples_idx = np.arange(median_frames.shape[0])
    peak_amplitudes = np.column_stack([
        signal[samples_idx, peak_idxs[:, 0]],
        signal[samples_idx, peak_idxs[:, 1]]
    ])
    
    return peak_amplitudes


def get_signal_variance(X: np.ndarray, idx: int):
    """
    Compute variance of signal across slow time at specified sample index.

    Args:
        X (np.ndarray):         Input radar data (samples, fast_time, slow_time)
        idx (int):              Sample index to compute variance for

    Returns:
        variance (np.ndarray):  Variance for each fast time bin
    """
    
    signal = np.abs(X[idx, :, :])
    variance = np.var(signal, axis=1)
    return variance


def get_peak_variance(X: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract variance of the two largest peaks for each sample.

    Args:
        X (np.ndarray):         Input radar data of shape (samples, fast_time, slow_time)
        peak_idxs (np.ndarray): Peak indices of shape (samples, 2)

    Returns:
        variance (np.ndarray):  Peak variances of shape (samples, 2)
    """
    
    logger.info("Computing peak variances...")
    
    variance = np.zeros((X.shape[0], 2))
    
    for i in range(X.shape[0]):
        signal_var = get_signal_variance(X, i)
        variance[i, 0] = signal_var[peak_idxs[i, 0]]
        variance[i, 1] = signal_var[peak_idxs[i, 1]]
        
    return variance


def get_signal_entropy(X: np.ndarray, idx: int):
    """
    Compute entropy of signal at specified sample index.

    Args:
        X (np.ndarray):                 Input radar data (samples, fast_time, slow_time)
        idx (int):                      Sample index to compute entropy for

    Returns:
        entropy_values (np.ndarray):    Entropy for each fast time bin
    """
    
    signal = np.abs(X[idx, :, :])
    entropy_values = np.zeros(signal.shape[0])
    
    for i in range(signal.shape[0]):
        # Normalize to create probability distribution
        prob_dist = signal[i] / (np.sum(signal[i]) + 1e-10)
        # Calculate entropy, avoiding log(0)
        entropy_values[i] = -np.sum(prob_dist * np.log2(prob_dist + 1e-10))
            
    return entropy_values


def get_peak_entropy(X: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract entropy of the two largest peaks for each sample.

    Args:
        X (np.ndarray):         Input radar data of shape (samples, fast_time, slow_time)
        peak_idxs (np.ndarray): Peak indices of shape (samples, 2)

    Returns:
        entropy_values (np.ndarray): Peak entropies of shape (samples, 2)
    """
    logger.info("Computing peak entropies...")
    
    entropy_values = np.zeros((X.shape[0], 2))
    
    for i in range(X.shape[0]):
        signal_entropy = get_signal_entropy(X, i)
        entropy_values[i, 0] = signal_entropy[peak_idxs[i, 0]]
        entropy_values[i, 1] = signal_entropy[peak_idxs[i, 1]]
        
    return entropy_values


def get_peak_delay(peak_idxs: np.ndarray):
    """
    Extract timing information for the two largest peaks. 

    Args:
        peak_idxs (np.ndarray): Peak indices of shape (samples, 2)

    Returns: 
        delays (np.ndarray): Peak delays of shape (samples, 3)
                             [delay_to_peak1, delay_to_peak2, delay_difference]
    """
    
    logger.info("Computing peak delays...")
    
    delays = np.zeros((peak_idxs.shape[0], 3))
    
    # Fully vectorized delay computation
    delays[:, 0] = peak_idxs[:, 0]  # First peak delay
    delays[:, 1] = peak_idxs[:, 1]  # Second peak delay
    delays[:, 2] = peak_idxs[:, 1] - peak_idxs[:, 0]  # Delay difference
    
    return delays


def get_peak_width(median_frames: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract width of the two largest peaks.

    Args:
        median_frames (np.ndarray): Median frames of shape (samples, fast_time)
        peak_idxs (np.ndarray):     Peak indices of shape (samples, 2)

    Returns:
        widths (np.ndarray): Peak widths of shape (samples, 2)
    """
    
    logger.info("Computing peak widths...")
    
    signal = np.abs(median_frames)
    widths = np.zeros((median_frames.shape[0], 2))
    
    for i in range(median_frames.shape[0]):
        try:
            width_1 = peak_widths(signal[i], [peak_idxs[i, 0]])[0][0]
            width_2 = peak_widths(signal[i], [peak_idxs[i, 1]])[0][0]
            widths[i, 0] = width_1
            widths[i, 1] = width_2
        except (IndexError, ValueError):
            # Use default width if calculation fails
            widths[i] = [1.0, 1.0]
            
    return widths


def get_peak_shape_stats(median_frames: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract skewness and kurtosis of peak regions.

    Args:
        median_frames (np.ndarray): Median frames of shape (samples, fast_time)
        peak_idxs (np.ndarray):     Peak indices of shape (samples, 2)

    Returns:
        skewness (np.ndarray): Skewness of peak (samples, 2)
        kurtosis (np.ndarray): Kurtosis of peak (samples, 2)
    """
    
    logger.info("Computing peak shape statistics...")
    
    skewness = np.zeros((median_frames.shape[0], 2))
    kurt = np.zeros((median_frames.shape[0], 2))
    signal = np.abs(median_frames)
    
    for i in range(median_frames.shape[0]):
        try:
            # Define regions around peaks for shape analysis
            peak1_end = min(peak_idxs[i, 0] + 1, signal.shape[1])
            peak2_start = max(peak_idxs[i, 1], 0)
            
            peak1_region = signal[i, :peak1_end] if peak1_end > 2 else signal[i, :5]
            peak2_region = signal[i, peak2_start:] if peak2_start < signal.shape[1] - 2 else signal[i, -5:]
            
            # Ensure minimum region size for valid statistics
            if len(peak1_region) >= 3:
                skewness[i, 0] = skew(peak1_region)
                kurt[i, 0] = kurtosis(peak1_region)
            
            if len(peak2_region) >= 3:
                skewness[i, 1] = skew(peak2_region)
                kurt[i, 1] = kurtosis(peak2_region)
                
        except (ValueError, RuntimeWarning):
            # Use default values if calculation fails
            skewness[i] = [0.0, 0.0]
            kurt[i] = [0.0, 0.0]
            
    return skewness, kurt


def get_signal_energy(X: np.ndarray, scan_idx: int = 0, spec_idx: int = 0):
    """
    Computes the energy of the signal at scan_idx and spec_idx.

    Args:
        X (np.ndarray):         Input radar data (samples, fast_time, slow_time)
        scan_idx (int):         Index of the sample to compute the energy for.
        spec_idx (int):         Index of the spectrum to compute the energy for (in fast time).
        
    Returns:
        energy (float):         Signal energy (sum of squares over slow time)
    """
    
    signal = np.abs(X[scan_idx, :, :])[spec_idx]
    return np.sum(signal ** 2)


def get_peak_signal_energy(X: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract energy of the two largest peaks for each sample.
    
    Args:
        X (np.ndarray):         Input radar data of shape (samples, fast_time, slow_time)
        peak_idxs (np.ndarray): Peak indices of shape (samples, 2)

    Returns:
        energy (np.ndarray):    Energy of the two largest peaks for each scan (samples, 2)
    """
    
    logger.info("Computing peak signal energies...")
    
    energy = np.zeros((X.shape[0], 2))
    for i in range(X.shape[0]):
        energy[i, 0] = get_signal_energy(X, i, peak_idxs[i, 0])
        energy[i, 1] = get_signal_energy(X, i, peak_idxs[i, 1])
    return energy


def get_decay_rate(median_frames: np.ndarray, peak_idxs: np.ndarray, decay_points: int = 10):
    """
    Extract decay rate of peaks.

    Args:
        median_frames (np.ndarray): Median frames of shape (samples, fast_time)
        peak_idxs (np.ndarray):     Peak indices of shape (samples, 2)
        decay_points (int):         Number of points to use for slope fitting

    Returns:
        slopes (np.ndarray):        Decay rates of shape (samples, 2)
    """
    
    logger.info("Computing peak decay rates...")
    
    signal = np.abs(median_frames)
    slopes = np.zeros((median_frames.shape[0], 2))

    for i in range(median_frames.shape[0]):
        for peak_num in range(2):
            try:
                start = peak_idxs[i, peak_num]
                end = min(signal.shape[1], start + decay_points)
                
                if end - start >= 2:  # Need at least 2 points for fitting
                    x = np.arange(start, end)
                    y = signal[i, start:end]
                    slope, _ = np.polyfit(x, y, 1)
                    slopes[i, peak_num] = slope
                    
            except (ValueError, np.linalg.LinAlgError):
                slopes[i, peak_num] = 0.0

    return slopes


def get_ascend_rate(median_frames: np.ndarray, peak_idxs: np.ndarray, ascend_points: int = 10):
    """
    Extract ascent rate of peaks.

    Args:
        median_frames (np.ndarray): Median frames of shape (samples, fast_time)
        peak_idxs (np.ndarray):     Peak indices of shape (samples, 2)
        ascend_points (int):        Number of points to use for slope fitting

    Returns:
        slopes (np.ndarray):        Ascent rates of shape (samples, 2)
    """
    
    logger.info("Computing peak ascent rates...")
    
    signal = np.abs(median_frames)
    slopes = np.zeros((median_frames.shape[0], 2))

    for i in range(median_frames.shape[0]):
        for peak_num in range(2):
            try:
                end = peak_idxs[i, peak_num]
                start = max(0, end - ascend_points)
                
                if end - start >= 2:  # Need at least 2 points for fitting
                    x = np.arange(start, end)
                    y = signal[i, start:end]
                    slope, _ = np.polyfit(x, y, 1)
                    slopes[i, peak_num] = slope
                    
            except (ValueError, np.linalg.LinAlgError):
                slopes[i, peak_num] = 0.0

    return slopes

def amplitude_features(X: np.ndarray):
    """
    Extract all amplitude-related features efficiently.
    
    Args:
        X (np.ndarray):         Input radar data of shape (samples, fast_time, slow_time)
        
    Returns:
        feature_table (pd.DataFrame): Feature table with amplitude features
    """
    
    logger.info("Extracting amplitude features...")
    
    # Initialize feature table
    feature_table = pd.DataFrame()
    
    # Get common components
    median_frames = get_median_frames(X=X)
    peak_idxs = get_peak_idx(median_frames)
    
    # Extract all features at once to minimize redundant computations
    peak_amplitude = get_peak_amplitude(median_frames, peak_idxs)
    peak_variance = get_peak_variance(X, peak_idxs)
    peak_entropy = get_peak_entropy(X, peak_idxs)
    peak_delay = get_peak_delay(peak_idxs)
    peak_width = get_peak_width(median_frames, peak_idxs)
    peak_skew, peak_kurt = get_peak_shape_stats(median_frames, peak_idxs)
    peak_energy = get_peak_signal_energy(X, peak_idxs)
    decay_rate = get_decay_rate(median_frames, peak_idxs)
    ascend_rate = get_ascend_rate(median_frames, peak_idxs)

    # Add features to table
    feature_table['Peak Amplitude 1'] = peak_amplitude[:, 0]
    feature_table['Peak Amplitude 2'] = peak_amplitude[:, 1]
    feature_table['Peak Variance 1'] = peak_variance[:, 0]
    feature_table['Peak Variance 2'] = peak_variance[:, 1]
    feature_table['Peak Entropy 1'] = peak_entropy[:, 0]
    feature_table['Peak Entropy 2'] = peak_entropy[:, 1]
    
    # Compute ratio features with numerical stability
    amplitude2variance_ratio = peak_amplitude / (peak_variance + 1e-10)
    amplitude2entropy_ratio = peak_amplitude / (peak_entropy + 1e-10)
    
    feature_table['Peak Amplitude to Variance Ratio 1'] = amplitude2variance_ratio[:, 0]
    feature_table['Peak Amplitude to Variance Ratio 2'] = amplitude2variance_ratio[:, 1]
    feature_table['Peak Amplitude to Entropy Ratio 1'] = amplitude2entropy_ratio[:, 0]
    feature_table['Peak Amplitude to Entropy Ratio 2'] = amplitude2entropy_ratio[:, 1]
    
    feature_table['Peak Delay 1'] = peak_delay[:, 0]
    feature_table['Peak Delay 2'] = peak_delay[:, 1]
    feature_table['Peak Delay Difference'] = peak_delay[:, 2]
    feature_table['Peak Width 1'] = peak_width[:, 0]
    feature_table['Peak Width 2'] = peak_width[:, 1]
    feature_table['Peak Skewness 1'] = peak_skew[:, 0]
    feature_table['Peak Skewness 2'] = peak_skew[:, 1]
    feature_table['Peak Kurtosis 1'] = peak_kurt[:, 0]
    feature_table['Peak Kurtosis 2'] = peak_kurt[:, 1]
    feature_table['Peak Energy 1'] = peak_energy[:, 0]
    feature_table['Peak Energy 2'] = peak_energy[:, 1]
    feature_table['Decay Rate 1'] = decay_rate[:, 0]
    feature_table['Decay Rate 2'] = decay_rate[:, 1]
    feature_table['Ascend Rate 1'] = ascend_rate[:, 0]
    feature_table['Ascend Rate 2'] = ascend_rate[:, 1]
    
    return feature_table