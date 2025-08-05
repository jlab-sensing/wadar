"""
Phase-based feature engineering tools for radar signal processing. 
"""

import logging
logger = logging.getLogger(__name__)

import numpy as np
import pandas as pd
from .amplitude_features import get_median_frames, get_peak_idx


def get_median_frames_phase(X: np.ndarray):
    """
    Get the median frame across slow time for phase calculations.

    Parameters:
        X: Raw data (samples, fast time, slow time)

    Returns:
        np.ndarray: Median frames of shape (samples, fast_time)
    """
    
    logger.info("Computing median frame across slow time for phase features")
    
    median_frames = np.median(X, axis=2)
    
    return median_frames


def get_peak_phase(median_frames: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract phase of the two largest peaks using vectorized operations.

    Parameters:
        median_frames: Median frames of shape (samples, fast_time)
        peak_idxs: Peak indices of shape (samples, 2)

    Returns:
        np.ndarray: Peak phases of shape (samples, 2)
    """
    
    logger.info("Computing peak phases...")
    
    signal = np.angle(median_frames)
    
    # Vectorized phase extraction
    samples_idx = np.arange(median_frames.shape[0])
    peak_phases = np.column_stack([
        signal[samples_idx, peak_idxs[:, 0]],
        signal[samples_idx, peak_idxs[:, 1]]
    ])
    
    return peak_phases


def get_phase_variance(X: np.ndarray, idx: int):
    """
    Computes the variance of the phase of the signal at idx. Phase variance is the variance of the phase of the signal.
    We hypothesize that the phase variance is related to the soil condition, as it may indicate the stability of the signal.

    Parameters:
        X: Input radar data
        idx: Index of the scan to compute the phase variance for.
        
    Returns:
        np.ndarray: Phase variance for each fast time bin
    """
    
    signal = np.angle(X[idx, :, :])
    return np.var(signal, axis=1)


def get_peak_phase_variance(X: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract variance of the phase of the two largest peaks for each sample.

    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        peak_idxs: Peak indices of shape (samples, 2)

    Returns:
        np.ndarray: Variance of the phase of the two largest peaks for each scan.
    """
    
    logger.info("Computing peak phase variances...")
    
    phase_variance = np.zeros((X.shape[0], 2))
    for i in range(X.shape[0]):
        signal_phase_var = get_phase_variance(X, i)
        phase_variance[i, 0] = signal_phase_var[peak_idxs[i, 0]]
        phase_variance[i, 1] = signal_phase_var[peak_idxs[i, 1]]
    return phase_variance


def get_circularity_coefficient(X: np.ndarray, idx: int = 0):
    """
    Compute circularity coefficient with improved numerical stability.

    Parameters:
        X: Input radar data
        idx: Sample index to compute circularity coefficient for
        
    Returns:
        np.ndarray: Circularity coefficients for each fast time bin
    """
    
    signal = X[idx, :, :]
    coeffs = np.zeros(signal.shape[0])
    
    for i in range(signal.shape[0]):
        cur_signal = signal[i, :]
        
        if len(cur_signal) == 0:
            coeffs[i] = 0.0
            continue
            
        # Use conjugate for proper circularity calculation
        mean_conj_prod = np.mean(cur_signal * np.conj(cur_signal))
        mean_power = np.mean(np.abs(cur_signal) ** 2)
        
        if mean_power > 1e-10:
            coeffs[i] = np.abs(mean_conj_prod) / mean_power
        else:
            coeffs[i] = 0.0
            
    return coeffs


def get_peak_circularity_coefficient(X: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract circularity coefficient of the two largest peaks for each sample.

    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        peak_idxs: Peak indices of shape (samples, 2)

    Returns:
        np.ndarray: Circularity coefficient of the two largest peaks for each scan.
    """
    
    logger.info("Computing peak circularity coefficients...")
    
    coeffs = np.zeros((X.shape[0], 2))

    for i in range(X.shape[0]):
        circ_coeffs = get_circularity_coefficient(X, i)
        coeffs[i, 0] = circ_coeffs[peak_idxs[i, 0]]
        coeffs[i, 1] = circ_coeffs[peak_idxs[i, 1]]
    return coeffs


def get_phase_jitter(X: np.ndarray, scan_idx: int = 0, spec_idx: int = 0):
    """
    Compute phase jitter with improved error handling.

    Parameters:
        X: Input radar data
        scan_idx: Sample index
        spec_idx: Fast time index

    Returns:
        float: Phase jitter value
    """
    
    try:
        signal = X[scan_idx, spec_idx, :]
        if len(signal) < 2:
            return 0.0
            
        phase = np.unwrap(np.angle(signal))
        phase_diff = np.diff(phase)
        
        return float(np.var(phase_diff))
        
    except (IndexError, ValueError):
        return 0.0


def get_peak_phase_jitter(X: np.ndarray, peak_idxs: np.ndarray):
    """
    Extract phase jitter of the two largest peaks.

    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        peak_idxs: Peak indices of shape (samples, 2)

    Returns:
        np.ndarray: Peak phase jitters of shape (samples, 2)
    """
    
    logger.info("Computing peak phase jitters...")
    
    phase_jitter = np.zeros((X.shape[0], 2))
    
    for i in range(X.shape[0]):
        phase_jitter[i, 0] = get_phase_jitter(X, i, peak_idxs[i, 0])
        phase_jitter[i, 1] = get_phase_jitter(X, i, peak_idxs[i, 1])
        
    return phase_jitter


def phase_features(X: np.ndarray):
    """
    Extract all phase-related features.
    
    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        
    Returns:
        pd.DataFrame: Feature table with phase features
    """
    
    logger.info("Extracting phase features...")
    
    # Initialize feature table
    feature_table = pd.DataFrame()
    
    # Get common components
    median_frames = get_median_frames(X=X)
    peak_idxs = get_peak_idx(median_frames)
    
    # Extract all phase features at once
    peak_phase = get_peak_phase(median_frames, peak_idxs)
    phase_variance = get_peak_phase_variance(X, peak_idxs)
    circularity_coefficient = get_peak_circularity_coefficient(X, peak_idxs)
    phase_jitter = get_peak_phase_jitter(X, peak_idxs)

    # Add to feature table
    feature_table['Peak Phase 1'] = peak_phase[:, 0]
    feature_table['Peak Phase 2'] = peak_phase[:, 1]
    feature_table['Phase Variance 1'] = phase_variance[:, 0]
    feature_table['Phase Variance 2'] = phase_variance[:, 1]
    feature_table['Circularity Coefficient 1'] = circularity_coefficient[:, 0]
    feature_table['Circularity Coefficient 2'] = circularity_coefficient[:, 1]
    feature_table['Phase Jitter 1'] = phase_jitter[:, 0]
    feature_table['Phase Jitter 2'] = phase_jitter[:, 1]
    
    return feature_table
