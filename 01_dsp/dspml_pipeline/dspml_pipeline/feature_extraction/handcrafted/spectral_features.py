"""
Spectral domain feature engineering tools for radar signal processing. 
"""

import logging
logger = logging.getLogger(__name__)

import numpy as np
import pandas as pd
from .amplitude_features import get_median_frames, get_peak_idx


def get_spectral_centroid(X: np.ndarray):
    """
    Compute spectral centroid for each sample (frequency center of mass).
    
    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        
    Returns:
        np.ndarray: Spectral centroid for each sample
    """
    
    logger.info("Computing spectral centroids...")
    
    centroids = np.zeros(X.shape[0])
    
    for i in range(X.shape[0]):
        # Average across slow time, then take FFT
        avg_signal = np.mean(np.abs(X[i]), axis=1)
        
        # Compute FFT
        fft_signal = np.fft.fft(avg_signal)
        magnitude = np.abs(fft_signal[:len(fft_signal)//2])
        
        # Frequency bins
        freqs = np.arange(len(magnitude))
        
        # Compute centroid
        if np.sum(magnitude) > 0:
            centroids[i] = np.sum(freqs * magnitude) / np.sum(magnitude)
        else:
            centroids[i] = 0.0
            
    return centroids


def get_spectral_bandwidth(X: np.ndarray):
    """
    Compute spectral bandwidth for each sample.
    
    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        
    Returns:
        np.ndarray: Spectral bandwidth for each sample
    """
    
    logger.info("Computing spectral bandwidths...")
    
    bandwidths = np.zeros(X.shape[0])
    
    for i in range(X.shape[0]):
        avg_signal = np.mean(np.abs(X[i]), axis=1)
        fft_signal = np.fft.fft(avg_signal)
        magnitude = np.abs(fft_signal[:len(fft_signal)//2])
        
        freqs = np.arange(len(magnitude))
        
        if np.sum(magnitude) > 0:
            centroid = np.sum(freqs * magnitude) / np.sum(magnitude)
            bandwidth = np.sqrt(np.sum(((freqs - centroid) ** 2) * magnitude) / np.sum(magnitude))
            bandwidths[i] = bandwidth
        else:
            bandwidths[i] = 0.0
            
    return bandwidths


def get_spectral_rolloff(X: np.ndarray, rolloff_percent: float = 0.85):
    """
    Compute spectral rolloff frequency - the frequency below which a given percentage 
    of the total spectral energy is contained.
    
    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        rolloff_percent: Percentage of energy for rolloff calculation
        
    Returns:
        np.ndarray: Spectral rolloff for each sample
    """
    
    logger.info("Computing spectral rolloffs...")
    
    rolloffs = np.zeros(X.shape[0])
    
    for i in range(X.shape[0]):
        avg_signal = np.mean(np.abs(X[i]), axis=1)
        fft_signal = np.fft.fft(avg_signal)
        magnitude = np.abs(fft_signal[:len(fft_signal)//2])
        
        cumsum_mag = np.cumsum(magnitude)
        total_energy = cumsum_mag[-1]
        
        if total_energy > 0:
            rolloff_idx = np.where(cumsum_mag >= rolloff_percent * total_energy)[0]
            rolloffs[i] = rolloff_idx[0] if len(rolloff_idx) > 0 else len(magnitude) - 1
        else:
            rolloffs[i] = 0.0
            
    return rolloffs


def get_spectral_flatness(X: np.ndarray):
    """
    Compute spectral flatness (Wiener entropy) - measure of how noise-like vs. tonal the spectrum is.
    Low values indicate tonal signals, high values indicate noise-like signals.
    
    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        
    Returns:
        np.ndarray: Spectral flatness for each sample
    """
    
    logger.info("Computing spectral flatness...")
    
    flatness = np.zeros(X.shape[0])
    
    for i in range(X.shape[0]):
        avg_signal = np.mean(np.abs(X[i]), axis=1)
        fft_signal = np.fft.fft(avg_signal)
        magnitude = np.abs(fft_signal[:len(fft_signal)//2])
        
        # Avoid zeros
        magnitude = magnitude + 1e-10
        
        geometric_mean = np.exp(np.mean(np.log(magnitude)))
        arithmetic_mean = np.mean(magnitude)
        
        if arithmetic_mean > 0:
            flatness[i] = geometric_mean / arithmetic_mean
        else:
            flatness[i] = 0.0
            
    return flatness


def get_spectral_flux(X: np.ndarray):
    """
    Compute spectral flux - measure of how quickly the power spectrum changes.
    Can indicate material transitions or boundaries.
    
    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        
    Returns:
        np.ndarray: Spectral flux for each sample
    """
    
    logger.info("Computing spectral flux...")
    
    flux = np.zeros(X.shape[0])
    
    for i in range(X.shape[0]):
        # Compute magnitude spectrum for each slow time sample
        slow_time_spectra = []
        for j in range(X.shape[2]):
            fft_signal = np.fft.fft(X[i, :, j])
            magnitude = np.abs(fft_signal[:len(fft_signal)//2])
            slow_time_spectra.append(magnitude)
        
        # Compute flux as sum of squared differences between consecutive spectra
        if len(slow_time_spectra) > 1:
            flux_sum = 0.0
            for j in range(1, len(slow_time_spectra)):
                diff = slow_time_spectra[j] - slow_time_spectra[j-1]
                flux_sum += np.sum(diff ** 2)
            flux[i] = flux_sum / (len(slow_time_spectra) - 1)
        else:
            flux[i] = 0.0
            
    return flux

# TODO: The dominant frequency

def spectral_features(X: np.ndarray):
    """
    Extract all spectral domain features.
    
    Parameters:
        X: Input radar data of shape (samples, fast_time, slow_time)
        
    Returns:
        pd.DataFrame: Feature table with spectral features
    """
    
    logger.info("Extracting spectral features...")
    
    # Initialize feature table
    feature_table = pd.DataFrame()
    
    # Extract all spectral features
    spectral_centroid = get_spectral_centroid(X)
    spectral_bandwidth = get_spectral_bandwidth(X)
    spectral_rolloff = get_spectral_rolloff(X)
    spectral_flatness = get_spectral_flatness(X)
    spectral_flux = get_spectral_flux(X)
    
    # Add to feature table
    feature_table['Spectral Centroid'] = spectral_centroid
    feature_table['Spectral Bandwidth'] = spectral_bandwidth
    feature_table['Spectral Rolloff'] = spectral_rolloff
    feature_table['Spectral Flatness'] = spectral_flatness
    feature_table['Spectral Flux'] = spectral_flux
    
    return feature_table
