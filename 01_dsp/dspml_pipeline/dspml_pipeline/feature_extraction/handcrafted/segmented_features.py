"""
Segmented and depth-based feature engineering tools for radar signal processing.
"""

import logging
logger = logging.getLogger(__name__)

import numpy as np
import pandas as pd
from scipy.stats import entropy, skew
from .amplitude_features import get_median_frames, get_peak_idx


def get_segment_entropy(X: np.ndarray, n_segments: int = 5):
    """
    Compute entropy for different depth segments.
    
    Parameters:
        X (np.ndarray):                     Input radar data of shape (samples, fast_time, slow_time)
        n_segments (int):                   Number of depth segments to analyze
        
    Returns:
        segment_entropies (np.ndarray):     Entropy features for each sample (n_segments columns)
    """
    
    logger.info("Computing segment entropy...")
    
    segment_entropies = np.zeros((X.shape[0], n_segments))
    
    for i in range(X.shape[0]):
        # Get amplitude data
        amp_data = np.abs(X[i])
        
        # Divide into depth segments
        segment_size = amp_data.shape[0] // n_segments
        
        for seg in range(n_segments):
            start_idx = seg * segment_size
            end_idx = (seg + 1) * segment_size if seg < n_segments - 1 else amp_data.shape[0]
            
            # Extract segment
            segment_data = amp_data[start_idx:end_idx, :]
            
            # Flatten and compute histogram for entropy
            flat_data = segment_data.flatten()
            if len(flat_data) > 0:
                hist, _ = np.histogram(flat_data, bins=50, density=True)
                hist = hist + 1e-10  # Avoid zeros
                segment_entropies[i, seg] = entropy(hist)
            else:
                segment_entropies[i, seg] = 0.0
                
    return segment_entropies


def get_segment_energy(X: np.ndarray, n_segments: int = 5):
    """
    Compute energy (sum of squares) for different depth segments.
    
    Parameters:
        X (np.ndarray):                     Input radar data of shape (samples, fast_time, slow_time)
        n_segments (int):                   Number of depth segments to analyze
        
    Returns:
        segment_energies (np.ndarray):      Energy features for each sample (n_segments columns)
    """
    
    logger.info("Computing segment energy...")
    
    segment_energies = np.zeros((X.shape[0], n_segments))
    
    for i in range(X.shape[0]):
        amp_data = np.abs(X[i])
        segment_size = amp_data.shape[0] // n_segments
        
        for seg in range(n_segments):
            start_idx = seg * segment_size
            end_idx = (seg + 1) * segment_size if seg < n_segments - 1 else amp_data.shape[0]
            
            segment_data = amp_data[start_idx:end_idx, :]
            segment_energies[i, seg] = np.sum(segment_data ** 2)
            
    return segment_energies


def get_segment_variance(X: np.ndarray, n_segments: int = 5):
    """
    Compute variance for different depth segments.
    
    Parameters:
        X (np.ndarray):                     Input radar data of shape (samples, fast_time, slow_time)
        n_segments (int):                   Number of depth segments to analyze
        
    Returns:
        segment_variances (np.ndarray):     Variance features for each sample (n_segments columns)
    """
    
    logger.info("Computing segment variance...")
    
    segment_variances = np.zeros((X.shape[0], n_segments))
    
    for i in range(X.shape[0]):
        amp_data = np.abs(X[i])
        segment_size = amp_data.shape[0] // n_segments
        
        for seg in range(n_segments):
            start_idx = seg * segment_size
            end_idx = (seg + 1) * segment_size if seg < n_segments - 1 else amp_data.shape[0]
            
            segment_data = amp_data[start_idx:end_idx, :]
            segment_variances[i, seg] = np.var(segment_data)
            
    return segment_variances


def get_segment_coherence(X: np.ndarray, n_segments: int = 5):
    """
    Compute coherence for different depth segments.

    Parameters:
        X (np.ndarray):                     Input radar data of shape (samples, fast_time, slow_time)
        n_segments (int):                   Number of depth segments to analyze

    Returns:
        segment_coherences (np.ndarray):    Coherence features for each sample (n_segments columns)
    """
    
    logger.info("Computing segment coherence...")
    
    segment_coherences = np.zeros((X.shape[0], n_segments))
    
    for i in range(X.shape[0]):
        complex_data = X[i]
        segment_size = complex_data.shape[0] // n_segments
        
        for seg in range(n_segments):
            start_idx = seg * segment_size
            end_idx = (seg + 1) * segment_size if seg < n_segments - 1 else complex_data.shape[0]
            
            segment_data = complex_data[start_idx:end_idx, :]
            
            if segment_data.shape[1] > 1:
                # Compute phase consistency across slow time
                phases = np.angle(segment_data)
                phase_diff = np.diff(phases, axis=1)
                coherence = np.abs(np.mean(np.exp(1j * phase_diff)))
                segment_coherences[i, seg] = coherence
            else:
                segment_coherences[i, seg] = 1.0
                
    return segment_coherences


def get_segment_skewness(X: np.ndarray, n_segments: int = 5):
    """
    Compute skewness for different depth segments.
    
    Parameters:
        X (np.ndarray):                     Input radar data of shape (samples, fast_time, slow_time)
        n_segments (int):                   Number of depth segments to analyze
        
    Returns:
        segment_skewness (np.ndarray):      Skewness features for each sample (n_segments columns)
    """
    
    logger.info("Computing segment skewness...")
    
    segment_skewness = np.zeros((X.shape[0], n_segments))
    
    for i in range(X.shape[0]):
        amp_data = np.abs(X[i])
        segment_size = amp_data.shape[0] // n_segments
        
        for seg in range(n_segments):
            start_idx = seg * segment_size
            end_idx = (seg + 1) * segment_size if seg < n_segments - 1 else amp_data.shape[0]
            
            segment_data = amp_data[start_idx:end_idx, :]
            flat_data = segment_data.flatten()
            
            if len(flat_data) > 0:
                segment_skewness[i, seg] = skew(flat_data)
            else:
                segment_skewness[i, seg] = 0.0
                
    return segment_skewness


def get_segment_peak_density(X: np.ndarray, n_segments: int = 5):
    """
    Compute peak density for different depth segments.
    
    Parameters:
        X (np.ndarray):                         Input radar data of shape (samples, fast_time, slow_time)
        n_segments (int):                       Number of depth segments to analyze
        
    Returns:
        segment_peak_densities (np.ndarray):    Peak density features for each sample (n_segments columns)
    """
    
    logger.info("Computing segment peak density...")
    
    segment_peak_densities = np.zeros((X.shape[0], n_segments))
    
    for i in range(X.shape[0]):
        amp_data = np.abs(X[i])
        segment_size = amp_data.shape[0] // n_segments
        
        for seg in range(n_segments):
            start_idx = seg * segment_size
            end_idx = (seg + 1) * segment_size if seg < n_segments - 1 else amp_data.shape[0]
            
            segment_data = amp_data[start_idx:end_idx, :]
            
            # Count peaks in the averaged segment
            avg_segment = np.mean(segment_data, axis=1)
            
            # Simple peak detection (above mean + std)
            threshold = np.mean(avg_segment) + np.std(avg_segment)
            peaks = avg_segment > threshold
            peak_count = np.sum(peaks)
            
            # Normalize by segment length
            segment_peak_densities[i, seg] = peak_count / len(avg_segment)
            
    return segment_peak_densities


def get_attenuation_coefficient(X: np.ndarray):
    """
    Estimate attenuation coefficient from signal decay with depth.
    
    Parameters:
        X (np.ndarray):                     Input radar data of shape (samples, fast_time, slow_time)
        
    Returns:
        attenuation_coeffs (np.ndarray):    Attenuation coefficient for each sample (1D array)
    """
    
    logger.info("Computing attenuation coefficients...")
    
    attenuation_coeffs = np.zeros(X.shape[0])
    
    for i in range(X.shape[0]):
        # Average amplitude across slow time
        amp_profile = np.mean(np.abs(X[i]), axis=1)
        
        # Smooth to reduce noise
        if len(amp_profile) > 5:
            kernel = np.ones(5) / 5
            amp_profile = np.convolve(amp_profile, kernel, mode='same')
        
        # Fit exponential decay (log-linear relationship)
        depth_indices = np.arange(len(amp_profile))
        
        # Avoid zeros and negative values
        amp_profile = np.maximum(amp_profile, 1e-10)
        log_amp = np.log(amp_profile)
        
        # Linear fit to log amplitude vs depth
        if len(depth_indices) > 1:
            coeffs = np.polyfit(depth_indices, log_amp, 1)
            attenuation_coeffs[i] = -coeffs[0]  # Negative slope indicates attenuation
        else:
            attenuation_coeffs[i] = 0.0
            
    return attenuation_coeffs


def segmented_features(X: np.ndarray, n_segments: int = 5):
    """
    Extract segmented and depth-based features from radar data.

    Parameters:
        X (np.ndarray):                 Input radar data of shape (samples, fast_time, slow_time)
        n_segments (int):               Number of depth segments to analyze

    Returns:
        feature_table (pd.DataFrame):   Feature table with segmented and attenuation features for each sample
    """
    
    logger.info("Extracting segmented features...")
    
    # Initialize feature table
    feature_table = pd.DataFrame()
    
    # Extract all segmented features
    segment_entropy = get_segment_entropy(X, n_segments)
    segment_energy = get_segment_energy(X, n_segments)
    segment_variance = get_segment_variance(X, n_segments)
    segment_coherence = get_segment_coherence(X, n_segments)
    segment_skewness = get_segment_skewness(X, n_segments)
    segment_peak_density = get_segment_peak_density(X, n_segments)
    attenuation_coeff = get_attenuation_coefficient(X)
    
    # Add segment-based features to table
    for seg in range(n_segments):
        feature_table[f'Segment {seg+1} Entropy'] = segment_entropy[:, seg]
        feature_table[f'Segment {seg+1} Energy'] = segment_energy[:, seg]
        feature_table[f'Segment {seg+1} Variance'] = segment_variance[:, seg]
        feature_table[f'Segment {seg+1} Coherence'] = segment_coherence[:, seg]
        feature_table[f'Segment {seg+1} Skewness'] = segment_skewness[:, seg]
        feature_table[f'Segment {seg+1} Peak Density'] = segment_peak_density[:, seg]
    
    # Add global attenuation feature
    feature_table['Attenuation Coefficient'] = attenuation_coeff
    
    return feature_table
