# Feature engineering tools for radar signal processing:
# - Peak Amplitude, Shape, and Timing Analysis
# - Signal Entropy and Variance Features
# - Phase Analysis and Circularity Metrics

import os
from typing import Optional, Tuple

import numpy as np
import pandas as pd
from scipy.stats import skew, kurtosis
from scipy.signal import find_peaks, peak_widths
from sklearn.linear_model import Lasso, LassoCV
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.feature_selection import mutual_info_regression


class FeatureTools:
    """
    Feature engineering tools for radar signal processing.
    
    Extracts comprehensive features from radar signals for soil analysis,
    including magnitude, phase, timing, and statistical features.
    
    Parameters:
        X (np.ndarray): Input radar data of shape (samples, fast_time, slow_time)
        verbose (bool): Enable verbose output for debugging
        cache_computations (bool): Cache expensive computations for efficiency
    """

    def __init__(self, X: np.ndarray, verbose: bool = True, cache_computations: bool = True):
        """Initialize the FeatureTools with input data and configuration."""
        if not isinstance(X, np.ndarray) or X.ndim != 3:
            raise ValueError(f"Input X must be 3D numpy array, got {type(X)} with shape {getattr(X, 'shape', 'unknown')}")
        
        self.X = X
        self.verbose = verbose
        self.cache_computations = cache_computations
        self.feature_table = None
        
        # Cache for expensive computations
        self._cached_average_frame = None
        self._cached_peak_indices = None
        
        if self.verbose:
            print(f"[INFO] FeatureTools initialized with data shape: {X.shape}")

    def _info(self, message: str) -> None:
        """
        Print info message if verbose mode is enabled.
        """
        if self.verbose:
            print(f"[INFO] {message}")

    def _average_frame(self) -> np.ndarray:
        """
        Compute the average frame of the signal across slow time with caching.

        Returns:
            np.ndarray: Average frame of shape (samples, fast_time)
        """
        if self.cache_computations and self._cached_average_frame is not None:
            return self._cached_average_frame
            
        self._info("Computing average frame across slow time...")
        avg_frame = np.median(self.X, axis=2)
        
        if self.cache_computations:
            self._cached_average_frame = avg_frame
            
        return avg_frame
    
    def _get_peak_idx(self) -> np.ndarray:
        """
        Compute indices of the two largest peaks with improved error handling and caching.

        Returns:
            np.ndarray: Peak indices of shape (samples, 2)
        """
        if self.cache_computations and self._cached_peak_indices is not None:
            return self._cached_peak_indices
            
        self._info("Computing peak indices for all samples...")
        
        peak_idxs = np.zeros((self.X.shape[0], 2), dtype=int)
        average_signals = np.abs(self._average_frame())
        
        for i in range(self.X.shape[0]):
            signal = average_signals[i]
            
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
        
        if self.cache_computations:
            self._cached_peak_indices = peak_idxs
            
        return peak_idxs

    def peak_amplitude(self) -> np.ndarray:
        """
        Extract amplitude of the two largest peaks using vectorized operations.

        Returns:
            np.ndarray: Peak amplitudes of shape (samples, 2)
        """

        self._info("Extracting peak amplitudes...")
        
        peak_idxs = self._get_peak_idx()
        signal = np.abs(self._average_frame())
        
        # Vectorized peak amplitude extraction
        samples_idx = np.arange(self.X.shape[0])
        peak_amplitudes = np.column_stack([
            signal[samples_idx, peak_idxs[:, 0]],
            signal[samples_idx, peak_idxs[:, 1]]
        ])
        
        return peak_amplitudes

    def _get_signal_variance(self, idx: int) -> np.ndarray:
        """
        Compute variance of signal across slow time at specified sample index.

        Parameters:
            idx: Sample index to compute variance for

        Returns:
            np.ndarray: Variance for each fast time bin
        """

        signal = np.abs(self.X[idx, :, :])
        return np.var(signal, axis=1)
        
    def peak_variance(self) -> np.ndarray:
        """
        Extract variance of the two largest peaks for each sample.

        Returns:
            np.ndarray: Peak variances of shape (samples, 2)
        """

        self._info("Computing peak variances...")
        
        peak_idxs = self._get_peak_idx()
        variance = np.zeros((self.X.shape[0], 2))
        
        for i in range(self.X.shape[0]):
            signal_var = self._get_signal_variance(i)
            variance[i, 0] = signal_var[peak_idxs[i, 0]]
            variance[i, 1] = signal_var[peak_idxs[i, 1]]
            
        return variance

    def _get_signal_entropy(self, idx: int) -> np.ndarray:
        """
        Compute entropy of signal at specified sample index with improved efficiency.

        Parameters:
            idx: Sample index to compute entropy for

        Returns:
            np.ndarray: Entropy for each fast time bin
        """

        signal = np.abs(self.X[idx, :, :])
        entropy_values = np.zeros(signal.shape[0])
        
        for i in range(signal.shape[0]):
            try:
                hist, _ = np.histogram(signal[i, :], bins=30, density=True)
                # Remove zero bins to avoid log(0)
                hist = hist[hist > 0]
                if len(hist) > 0:
                    entropy_values[i] = -np.sum(hist * np.log(hist + 1e-10))
                else:
                    entropy_values[i] = 0.0
            except (ValueError, RuntimeWarning):
                entropy_values[i] = 0.0
                
        return entropy_values

    def peak_entropy(self) -> np.ndarray:
        """
        Extract entropy of the two largest peaks for each sample.

        Returns:
            np.ndarray: Peak entropies of shape (samples, 2)
        """

        self._info("Computing peak entropies...")
        
        peak_idxs = self._get_peak_idx()
        entropy_values = np.zeros((self.X.shape[0], 2))
        
        for i in range(self.X.shape[0]):
            signal_entropy = self._get_signal_entropy(i)
            entropy_values[i, 0] = signal_entropy[peak_idxs[i, 0]]
            entropy_values[i, 1] = signal_entropy[peak_idxs[i, 1]]
            
        return entropy_values
    
    def peak_delay(self) -> np.ndarray:
        """
        Extract timing information for the two largest peaks.
        
        Returns timing delays and the difference between them, which can
        indicate subsurface layer characteristics.

        Returns: 
            np.ndarray: Peak delays of shape (samples, 3)
                       [delay_to_peak1, delay_to_peak2, delay_difference]
        """

        self._info("Computing peak delays...")
        
        peak_idxs = self._get_peak_idx() 
        delays = np.zeros((self.X.shape[0], 3))
        
        # Fully vectorized delay computation
        delays[:, 0] = peak_idxs[:, 0]  # First peak delay
        delays[:, 1] = peak_idxs[:, 1]  # Second peak delay
        delays[:, 2] = peak_idxs[:, 1] - peak_idxs[:, 0]  # Delay difference
        
        return delays

    def peak_width(self) -> np.ndarray:
        """
        Extract width of the two largest peaks with improved error handling.

        Returns:
            np.ndarray: Peak widths of shape (samples, 2)
        """

        self._info("Computing peak widths...")
        
        peak_idxs = self._get_peak_idx()
        signal = np.abs(self._average_frame())
        widths = np.zeros((self.X.shape[0], 2))
        
        for i in range(self.X.shape[0]):
            try:
                width_1 = peak_widths(signal[i], [peak_idxs[i, 0]])[0][0]
                width_2 = peak_widths(signal[i], [peak_idxs[i, 1]])[0][0]
                widths[i, 0] = width_1
                widths[i, 1] = width_2
            except (IndexError, ValueError):
                # Use default width if calculation fails
                widths[i] = [1.0, 1.0]
                
        return widths

    def peak_shape_stats(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract skewness and kurtosis of peak regions with improved robustness.

        Returns:
            Tuple of (skewness, kurtosis) arrays, each of shape (samples, 2)
        """

        self._info("Computing peak shape statistics...")
        
        skewness = np.zeros((self.X.shape[0], 2))
        kurt = np.zeros((self.X.shape[0], 2))
        peak_idxs = self._get_peak_idx()
        signal = np.abs(self._average_frame())
        
        for i in range(self.X.shape[0]):
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
    
    def _get_signal_energy(self, scan_idx=0, spec_idx=0):
        """
        Computes the energy of the signal at scan_idx and spec_idx. Signal energy is the sum of the squares of the signal
        taken over slow time.

        Parameters:
            scan_idx (int): Index of the sample to compute the energy for.
            spec_idx (int): Index of the spectrum to compute the energy for (in fast time).
        """

        signal = np.abs(self.X[scan_idx, :, :])[spec_idx]
        return np.sum(signal ** 2)
    
    def peak_signal_energy(self):
        """
        Returns:
            np.ndarray: Energy of the two largest peaks for each scan.
        """

        peak_idxs = self._get_peak_idx()
        energy = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            energy[i, 0] = self._get_signal_energy(i, peak_idxs[i, 0])
            energy[i, 1] = self._get_signal_energy(i, peak_idxs[i, 1])
        return energy
    
    def decay_rate(self, decay_points: int = 10) -> np.ndarray:
        """
        Extract decay rate of peaks with improved efficiency and error handling.

        Parameters:
            decay_points: Number of points to use for slope fitting

        Returns:
            np.ndarray: Decay rates of shape (samples, 2)
        """

        self._info("Computing peak decay rates...")
        
        peak_idxs = self._get_peak_idx()
        signal = np.abs(self._average_frame())
        slopes = np.zeros((self.X.shape[0], 2))

        for i in range(self.X.shape[0]):
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

    def ascend_rate(self, ascend_points: int = 10) -> np.ndarray:
        """
        Extract ascent rate of peaks with improved efficiency and error handling.

        Parameters:
            ascend_points: Number of points to use for slope fitting

        Returns:
            np.ndarray: Ascent rates of shape (samples, 2)
        """

        self._info("Computing peak ascent rates...")
        
        peak_idxs = self._get_peak_idx()
        signal = np.abs(self._average_frame())
        slopes = np.zeros((self.X.shape[0], 2))

        for i in range(self.X.shape[0]):
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
    
    def peak_phase(self) -> np.ndarray:
        """
        Extract phase of the two largest peaks using vectorized operations.

        Returns:
            np.ndarray: Peak phases of shape (samples, 2)
        """

        self._info("Computing peak phases...")
        
        peak_idxs = self._get_peak_idx()
        signal = np.angle(self._average_frame())
        
        # Vectorized phase extraction
        samples_idx = np.arange(self.X.shape[0])
        peak_phases = np.column_stack([
            signal[samples_idx, peak_idxs[:, 0]],
            signal[samples_idx, peak_idxs[:, 1]]
        ])
        
        return peak_phases
            
    def _get_phase_variance(self, idx):
        """
        Computes the variance of the phase of the signal at idx. Phase variance is the variance of the phase of the signal.
        We hypothesize that the phase variance is related to the soil condition, as it may indicate the stability of the signal.

        Parameters:
            idx (int):  Index of the scan to compute the phase variance for.
        """

        signal = np.angle(self.X[idx, :, :])
        return np.var(signal, axis=1)

    def peak_phase_variance(self):
        """
        Returns:
            np.ndarray: Variance of the phase of the two largest peaks for each scan.
        """

        peak_idxs = self._get_peak_idx()
        phase_variance = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            signal_phase_var = self._get_phase_variance(i)
            phase_variance[i, 0] = signal_phase_var[peak_idxs[i, 0]]
            phase_variance[i, 1] = signal_phase_var[peak_idxs[i, 1]]
        return phase_variance

    def _get_circularity_coefficient(self, idx: int = 0) -> np.ndarray:
        """
        Compute circularity coefficient with improved numerical stability.

        Parameters:
            idx: Sample index to compute circularity coefficient for
            
        Returns:
            np.ndarray: Circularity coefficients for each fast time bin
        """

        signal = self.X[idx, :, :]
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


    def peak_circularity_coefficient(self):
        """
        Returns:
            np.ndarray: Circularity coefficient of the two largest peaks for each scan.
        """

        peak_idxs = self._get_peak_idx()
        coeffs = np.zeros((self.X.shape[0], 2))

        for i in range(self.X.shape[0]):
            circ_coeffs = self._get_circularity_coefficient(i)
            coeffs[i, 0] = circ_coeffs[peak_idxs[i, 0]]
            coeffs[i, 1] = circ_coeffs[peak_idxs[i, 1]]
        return coeffs

    def _get_phase_jitter(self, scan_idx: int = 0, spec_idx: int = 0) -> float:
        """
        Compute phase jitter with improved error handling.

        Parameters:
            scan_idx: Sample index
            spec_idx: Fast time index

        Returns:
            float: Phase jitter value
        """

        try:
            signal = self.X[scan_idx, spec_idx, :]
            if len(signal) < 2:
                return 0.0
                
            phase = np.unwrap(np.angle(signal))
            phase_diff = np.diff(phase)
            
            return float(np.var(phase_diff))
            
        except (IndexError, ValueError):
            return 0.0

    def peak_phase_jitter(self) -> np.ndarray:
        """
        Extract phase jitter of the two largest peaks.

        Returns:
            np.ndarray: Peak phase jitters of shape (samples, 2)
        """

        self._info("Computing peak phase jitters...")
        
        peak_idxs = self._get_peak_idx()
        phase_jitter = np.zeros((self.X.shape[0], 2))
        
        for i in range(self.X.shape[0]):
            phase_jitter[i, 0] = self._get_phase_jitter(i, peak_idxs[i, 0])
            phase_jitter[i, 1] = self._get_phase_jitter(i, peak_idxs[i, 1])
            
        return phase_jitter
    
    def spectral_centroid(self) -> np.ndarray:
        """
        Compute spectral centroid for each sample (frequency center of mass).
        Higher bulk density may shift the spectral centroid.
        
        Returns:
            np.ndarray: Spectral centroid for each sample
        """
        
        centroids = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            # Average across slow time, then take FFT
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
            
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

    def spectral_bandwidth(self) -> np.ndarray:
        """
        Compute spectral bandwidth for each sample.
        
        Returns:
            np.ndarray: Spectral bandwidth for each sample
        """
        
        bandwidths = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
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

    def spectral_rolloff(self, rolloff_percent: float = 0.85) -> np.ndarray:
        """
        Compute spectral rolloff frequency - the frequency below which a given percentage 
        of the total spectral energy is contained.
        
        Returns:
            np.ndarray: Spectral rolloff for each sample
        """
        
        rolloffs = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
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

    def spectral_flatness(self) -> np.ndarray:
        """
        Compute spectral flatness (Wiener entropy) - measure of how noise-like vs. tonal the spectrum is.
        Low values indicate tonal signals, high values indicate noise-like signals.
        
        Returns:
            np.ndarray: Spectral flatness for each sample
        """
        
        flatness = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
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

    def spectral_flux(self) -> np.ndarray:
        """
        Compute spectral flux - measure of how quickly the power spectrum changes.
        Can indicate material transitions or boundaries.
        
        Returns:
            np.ndarray: Spectral flux for each sample
        """
        
        flux = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            # Compute magnitude spectrum for each slow time sample
            slow_time_spectra = []
            for j in range(self.X.shape[2]):
                fft_signal = np.fft.fft(self.X[i, :, j])
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

    def dominant_frequency_components(self, n_components: int = 3) -> np.ndarray:
        """
        Extract dominant frequency components and their relative strengths.
        
        Returns:
            np.ndarray: Dominant frequency indices for each sample
        """
        
        dominant_freqs = np.zeros((self.X.shape[0], n_components))
        
        for i in range(self.X.shape[0]):
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
            fft_signal = np.fft.fft(avg_signal)
            magnitude = np.abs(fft_signal[:len(fft_signal)//2])
            
            # Find top frequency components
            if len(magnitude) >= n_components:
                top_indices = np.argsort(magnitude)[-n_components:][::-1]
                dominant_freqs[i] = top_indices
            else:
                # Pad with zeros if not enough frequency bins
                available = len(magnitude)
                top_indices = np.argsort(magnitude)[::-1]
                dominant_freqs[i, :available] = top_indices
                dominant_freqs[i, available:] = 0
                
        return dominant_freqs

    def segment_entropy(self, start_bin: int, end_bin: int) -> np.ndarray:
        """
        Compute entropy for a specific depth segment.
        
        Returns:
            np.ndarray: Entropy for each sample in the segment
        """
        
        entropy_values = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            segment_data = np.abs(self.X[i, start_bin:end_bin, :])
            
            if segment_data.size > 0:
                # Flatten the segment and compute histogram
                flat_data = segment_data.flatten()
                hist, _ = np.histogram(flat_data, bins=30, density=True)
                hist = hist[hist > 0]  # Remove zero bins
                
                if len(hist) > 0:
                    entropy_values[i] = -np.sum(hist * np.log(hist + 1e-10))
                else:
                    entropy_values[i] = 0.0
            else:
                entropy_values[i] = 0.0
                
        return entropy_values

    def segment_energy(self, start_bin: int, end_bin: int) -> np.ndarray:
        """
        Compute energy for a specific depth segment.
        
        Returns:
            np.ndarray: Energy for each sample in the segment
        """
        
        energy_values = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            segment_data = np.abs(self.X[i, start_bin:end_bin, :])
            energy_values[i] = np.sum(segment_data ** 2)
            
        return energy_values

    def segment_variance(self, start_bin: int, end_bin: int) -> np.ndarray:
        """
        Compute variance for a specific depth segment.
        
        Returns:
            np.ndarray: Variance for each sample in the segment
        """
        
        variance_values = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            segment_data = np.abs(self.X[i, start_bin:end_bin, :])
            if segment_data.size > 0:
                variance_values[i] = np.var(segment_data)
            else:
                variance_values[i] = 0.0
                
        return variance_values

    def segment_coherence(self, start_bin: int, end_bin: int) -> np.ndarray:
        """
        Compute coherence for a specific depth segment across slow time.
        
        Returns:
            np.ndarray: Coherence for each sample in the segment
        """
        
        coherence_values = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            segment_data = self.X[i, start_bin:end_bin, :]
            
            if segment_data.shape[1] > 1:
                # Compute correlation between adjacent slow time samples
                correlations = []
                for j in range(segment_data.shape[1] - 1):
                    sig1 = np.abs(segment_data[:, j])
                    sig2 = np.abs(segment_data[:, j+1])
                    if len(sig1) > 1 and len(sig2) > 1:
                        corr = np.corrcoef(sig1, sig2)[0, 1]
                        if not np.isnan(corr):
                            correlations.append(abs(corr))  # Use absolute value
                
                coherence_values[i] = np.mean(correlations) if correlations else 0.0
            else:
                coherence_values[i] = 0.0
                
        return coherence_values

    def segment_skewness(self, start_bin: int, end_bin: int) -> np.ndarray:
        """
        Compute skewness for a specific depth segment.
        
        Returns:
            np.ndarray: Skewness for each sample in the segment
        """
        
        skewness_values = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            segment_data = np.abs(self.X[i, start_bin:end_bin, :])
            
            if segment_data.size > 2:  # Need at least 3 points for skewness
                flat_data = segment_data.flatten()
                try:
                    skewness_values[i] = skew(flat_data)
                except:
                    skewness_values[i] = 0.0
            else:
                skewness_values[i] = 0.0
                
        return skewness_values

    def segment_peak_density(self, start_bin: int, end_bin: int) -> np.ndarray:
        """
        Compute peak density (number of peaks per unit length) for a depth segment.
        
        Returns:
            np.ndarray: Peak density for each sample in the segment
        """
        
        peak_density_values = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            # Average across slow time for this segment
            segment_signal = np.mean(np.abs(self.X[i, start_bin:end_bin, :]), axis=1)
            
            if len(segment_signal) > 2:
                # Find peaks with adaptive threshold
                threshold = np.max(segment_signal) * 0.1
                peaks, _ = find_peaks(segment_signal, height=threshold, distance=2)
                peak_density_values[i] = len(peaks) / len(segment_signal)
            else:
                peak_density_values[i] = 0.0
                
        return peak_density_values

    def attenuation_coefficient(self) -> np.ndarray:
        """
        Estimate attenuation coefficient from signal decay with depth.
        
        Returns:
            np.ndarray: Attenuation coefficient for each sample
        """
        
        attenuation = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            # Average signal across slow time
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
            
            # Fit exponential decay (log-linear relationship)
            depths = np.arange(len(avg_signal))
            log_signal = np.log(avg_signal + 1e-10)
            
            # Linear fit to log(signal) vs depth
            if len(depths) > 1:
                try:
                    slope, _ = np.polyfit(depths, log_signal, 1)
                    attenuation[i] = -slope  # Negative slope indicates attenuation
                except (np.linalg.LinAlgError, ValueError):
                    attenuation[i] = 0.0
            else:
                attenuation[i] = 0.0
                
        return attenuation
    
    def autocorrelation_features(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Advanced DSP: Compute autocorrelation-based features.
        Autocorrelation can reveal periodic structures and layer characteristics.
        
        Returns:
            Tuple of (max_autocorr, autocorr_lag, autocorr_decay)
        """
        
        max_autocorr = np.zeros(self.X.shape[0])
        autocorr_lag = np.zeros(self.X.shape[0])
        autocorr_decay = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            # Average signal across slow time
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
            
            # Compute autocorrelation
            signal_len = len(avg_signal)
            if signal_len > 10:  # Need sufficient length
                # Normalize signal
                signal_norm = (avg_signal - np.mean(avg_signal)) / (np.std(avg_signal) + 1e-10)
                
                # Full autocorrelation
                autocorr = np.correlate(signal_norm, signal_norm, mode='full')
                autocorr = autocorr[signal_len-1:]  # Take positive lags only
                
                # Normalize by zero-lag value
                if autocorr[0] > 0:
                    autocorr = autocorr / autocorr[0]
                
                    # Find maximum autocorrelation (excluding zero lag)
                    if len(autocorr) > 1:
                        max_idx = np.argmax(autocorr[1:]) + 1
                        max_autocorr[i] = autocorr[max_idx]
                        autocorr_lag[i] = max_idx
                        
                        # Compute decay rate
                        if max_idx > 0 and len(autocorr) > max_idx + 5:
                            decay_region = autocorr[max_idx:max_idx+10]
                            if len(decay_region) > 1:
                                try:
                                    # Fit exponential decay
                                    x = np.arange(len(decay_region))
                                    log_vals = np.log(np.abs(decay_region) + 1e-10)
                                    slope, _ = np.polyfit(x, log_vals, 1)
                                    autocorr_decay[i] = -slope
                                except:
                                    autocorr_decay[i] = 0.0
        
        return max_autocorr, autocorr_lag, autocorr_decay

    def wavelet_energy_features(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Advanced DSP: Wavelet-based multi-resolution analysis.
        Uses discrete wavelet transform to analyze signal at different scales.
        
        Returns:
            Tuple of (low_freq_energy, mid_freq_energy, high_freq_energy)
        """
        
        low_freq_energy = np.zeros(self.X.shape[0])
        mid_freq_energy = np.zeros(self.X.shape[0])
        high_freq_energy = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            # Average signal across slow time
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
            
            # Simple wavelet-like decomposition using multiple filtering
            signal_len = len(avg_signal)
            
            if signal_len >= 8:  # Need minimum length for decomposition
                # Low-pass filter (approximation)
                low_pass_kernel = np.ones(4) / 4
                if signal_len >= len(low_pass_kernel):
                    low_freq = np.convolve(avg_signal, low_pass_kernel, mode='valid')
                    low_freq_energy[i] = np.sum(low_freq ** 2)
                
                # High-pass filter (detail)
                high_pass_kernel = np.array([-1, 1])
                if signal_len >= len(high_pass_kernel):
                    high_freq = np.convolve(avg_signal, high_pass_kernel, mode='valid')
                    high_freq_energy[i] = np.sum(high_freq ** 2)
                
                # Mid-frequency using bandpass-like operation
                if signal_len >= 6:
                    mid_pass_kernel = np.array([-1, 0, 2, 0, -1]) / 2
                    if signal_len >= len(mid_pass_kernel):
                        mid_freq = np.convolve(avg_signal, mid_pass_kernel, mode='valid')
                        mid_freq_energy[i] = np.sum(mid_freq ** 2)
        
        return low_freq_energy, mid_freq_energy, high_freq_energy

    def signal_complexity_features(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Advanced DSP: Signal complexity measures.
        Computes Higuchi fractal dimension and approximate entropy.
        
        Returns:
            Tuple of (fractal_dimension, approximate_entropy)
        """
        
        fractal_dim = np.zeros(self.X.shape[0])
        approx_entropy = np.zeros(self.X.shape[0])
        
        for i in range(self.X.shape[0]):
            avg_signal = np.mean(np.abs(self.X[i]), axis=1)
            
            # Higuchi Fractal Dimension (simplified version)
            if len(avg_signal) > 10:
                k_max = min(5, len(avg_signal) // 3)
                lk_values = []
                
                for k in range(1, k_max + 1):
                    lk = 0
                    n = len(avg_signal)
                    normalization = (n - 1) / (((n - 1) // k) * k)
                    
                    for m in range(k):
                        lm = 0
                        max_idx = (n - 1 - m) // k
                        for j in range(max_idx):
                            idx1 = m + j * k
                            idx2 = m + (j + 1) * k
                            if idx2 < len(avg_signal):
                                lm += abs(avg_signal[idx2] - avg_signal[idx1])
                        
                        if max_idx > 0:
                            lm = lm * normalization / k
                            lk += lm
                    
                    if k > 0:
                        lk_values.append(lk / k)
                
                # Compute fractal dimension
                if len(lk_values) > 1:
                    try:
                        k_vals = np.arange(1, len(lk_values) + 1)
                        log_k = np.log(k_vals)
                        log_lk = np.log(np.array(lk_values) + 1e-10)
                        slope, _ = np.polyfit(log_k, log_lk, 1)
                        fractal_dim[i] = -slope
                    except:
                        fractal_dim[i] = 1.0
                else:
                    fractal_dim[i] = 1.0
            
            # Approximate Entropy (simplified)
            if len(avg_signal) > 20:
                m = 2  # Pattern length
                r = 0.2 * np.std(avg_signal)  # Tolerance
                
                def _maxdist(xi, xj, m):
                    return max([abs(ua - va) for ua, va in zip(xi, xj)])
                
                def _phi(m):
                    patterns = np.array([avg_signal[i:i + m] for i in range(len(avg_signal) - m + 1)])
                    C = np.zeros(len(patterns))
                    
                    for i in range(len(patterns)):
                        template_i = patterns[i]
                        for j in range(len(patterns)):
                            if _maxdist(template_i, patterns[j], m) <= r:
                                C[i] += 1.0
                    
                    phi = np.mean(np.log(C / len(patterns) + 1e-10))
                    return phi
                
                try:
                    approx_entropy[i] = _phi(m) - _phi(m + 1)
                except:
                    approx_entropy[i] = 0.0
        
        return fractal_dim, approx_entropy
    
    # TODO: FFT and some time domain features. Maybe correlation between peaks?
    #       There are advanced radar clutter analysis techniques.
    #       Persistence, but that might be the same as variance.
    #       Definitely missing some phase features that I don't know about.
    #       With the feature pruning methods I've implemented, there's no such thing as "too many features".

    def initialize_feature_table(self) -> None:
        """
        Initialize an empty DataFrame to store extracted features.
        """

        self._info("Initializing feature table...")
        self.feature_table = pd.DataFrame()
    
    def magnitude_features(self) -> None:
        """
        Extract all magnitude-related features efficiently.
        """

        self._info("Extracting magnitude features...")

        # Extract all features at once to minimize redundant computations
        peak_amplitude = self.peak_amplitude()
        peak_variance = self.peak_variance()
        peak_entropy = self.peak_entropy()
        peak_delay = self.peak_delay()
        peak_width = self.peak_width()
        peak_skew, peak_kurt = self.peak_shape_stats()
        peak_energy = self.peak_signal_energy()
        decay_rate = self.decay_rate()
        ascend_rate = self.ascend_rate()

        # Add to feature table
        self.feature_table['Peak Amplitude 1'] = peak_amplitude[:, 0]
        self.feature_table['Peak Amplitude 2'] = peak_amplitude[:, 1]
        self.feature_table['Peak Variance 1'] = peak_variance[:, 0]
        self.feature_table['Peak Variance 2'] = peak_variance[:, 1]
        self.feature_table['Peak Entropy 1'] = peak_entropy[:, 0]
        self.feature_table['Peak Entropy 2'] = peak_entropy[:, 1]
        
        # Compute ratio features with numerical stability
        amplitude2variance_ratio = peak_amplitude / (peak_variance + 1e-10)
        amplitude2entropy_ratio = peak_amplitude / (peak_entropy + 1e-10)
        
        self.feature_table['Amplitude to Variance Ratio 1'] = amplitude2variance_ratio[:, 0]
        self.feature_table['Amplitude to Variance Ratio 2'] = amplitude2variance_ratio[:, 1]
        self.feature_table['Amplitude to Entropy Ratio 1'] = amplitude2entropy_ratio[:, 0]
        self.feature_table['Amplitude to Entropy Ratio 2'] = amplitude2entropy_ratio[:, 1]
        
        self.feature_table['Peak Delay 1'] = peak_delay[:, 0]
        self.feature_table['Peak Delay 2'] = peak_delay[:, 1]
        self.feature_table['Peak Delay Difference'] = peak_delay[:, 2]
        self.feature_table['Peak Width 1'] = peak_width[:, 0]
        self.feature_table['Peak Width 2'] = peak_width[:, 1]
        self.feature_table['Peak Skewness 1'] = peak_skew[:, 0]
        self.feature_table['Peak Skewness 2'] = peak_skew[:, 1]
        self.feature_table['Peak Kurtosis 1'] = peak_kurt[:, 0]
        self.feature_table['Peak Kurtosis 2'] = peak_kurt[:, 1]
        self.feature_table['Peak Energy 1'] = peak_energy[:, 0]
        self.feature_table['Peak Energy 2'] = peak_energy[:, 1]
        self.feature_table['Decay Rate 1'] = decay_rate[:, 0]
        self.feature_table['Decay Rate 2'] = decay_rate[:, 1]
        self.feature_table['Ascend Rate 1'] = ascend_rate[:, 0]
        self.feature_table['Ascend Rate 2'] = ascend_rate[:, 1]

    def phase_features(self) -> None:
        """
        Extract all phase-related features efficiently.
        """

        self._info("Extracting phase features...")

        # Extract all phase features at once
        peak_phase = self.peak_phase()
        phase_variance = self.peak_phase_variance()
        circularity_coefficient = self.peak_circularity_coefficient()
        phase_jitter = self.peak_phase_jitter()

        # Add to feature table
        self.feature_table['Peak Phase 1'] = peak_phase[:, 0]
        self.feature_table['Peak Phase 2'] = peak_phase[:, 1]
        self.feature_table['Phase Variance 1'] = phase_variance[:, 0]
        self.feature_table['Phase Variance 2'] = phase_variance[:, 1]
        self.feature_table['Circularity Coefficient 1'] = circularity_coefficient[:, 0]
        self.feature_table['Circularity Coefficient 2'] = circularity_coefficient[:, 1]
        self.feature_table['Phase Jitter 1'] = phase_jitter[:, 0]
        self.feature_table['Phase Jitter 2'] = phase_jitter[:, 1]

    def spectral_features(self) -> None:
        """
        Extract spectral domain features that may correlate with bulk density.
        """
        
        self._info("Extracting spectral features...")
        
        # Get spectral characteristics
        spectral_centroid = self.spectral_centroid()
        spectral_bandwidth = self.spectral_bandwidth()
        spectral_rolloff = self.spectral_rolloff()
        spectral_flatness = self.spectral_flatness()
        spectral_flux = self.spectral_flux()
        dominant_freq = self.dominant_frequency_components()
        
        self.feature_table['Spectral Centroid'] = spectral_centroid
        self.feature_table['Spectral Bandwidth'] = spectral_bandwidth
        self.feature_table['Spectral Rolloff'] = spectral_rolloff
        self.feature_table['Spectral Flatness'] = spectral_flatness
        self.feature_table['Spectral Flux'] = spectral_flux
        self.feature_table['Dominant Freq 1'] = dominant_freq[:, 0]
        self.feature_table['Dominant Freq 2'] = dominant_freq[:, 1]
        self.feature_table['Dominant Freq 3'] = dominant_freq[:, 2]

    def depth_segmented_features(self, segment_size: int = 100) -> None:
        """
        Extract features from different depth segments (range bins).
        """
        
        self._info(f"Extracting depth-segmented features (segment size: {segment_size})...")
        
        # Calculate number of segments based on fast time dimension
        fast_time_bins = self.X.shape[1]
        n_segments = max(1, fast_time_bins // segment_size)
        
        # Extract multiple features for each segment
        for seg_idx in range(min(n_segments, 4)):  # Limit to 4 segments to avoid too many features
            start_bin = seg_idx * segment_size
            end_bin = min((seg_idx + 1) * segment_size, fast_time_bins)
            
            entropy_seg = self.segment_entropy(start_bin, end_bin)
            energy_seg = self.segment_energy(start_bin, end_bin)
            variance_seg = self.segment_variance(start_bin, end_bin)
            coherence_seg = self.segment_coherence(start_bin, end_bin)
            skewness_seg = self.segment_skewness(start_bin, end_bin)
            peak_density_seg = self.segment_peak_density(start_bin, end_bin)
            
            self.feature_table[f'Segment_{seg_idx+1}_Entropy'] = entropy_seg
            self.feature_table[f'Segment_{seg_idx+1}_Energy'] = energy_seg
            self.feature_table[f'Segment_{seg_idx+1}_Variance'] = variance_seg
            self.feature_table[f'Segment_{seg_idx+1}_Coherence'] = coherence_seg
            self.feature_table[f'Segment_{seg_idx+1}_Skewness'] = skewness_seg
            self.feature_table[f'Segment_{seg_idx+1}_Peak_Density'] = peak_density_seg

    def attenuation_features(self) -> None:
        """
        Extract attenuation-related features that may indicate soil density.
        """
        
        self._info("Extracting attenuation features...")
        
        # Get attenuation characteristics
        attenuation_coeff = self.attenuation_coefficient()
        
        self.feature_table['Attenuation Coefficient'] = attenuation_coeff

    def advanced_dsp_features(self) -> None:
        """
        Extract advanced DSP features including autocorrelation, wavelet, and complexity measures.
        """
        
        self._info("Extracting advanced DSP features...")
        
        # Autocorrelation features
        max_autocorr, autocorr_lag, autocorr_decay = self.autocorrelation_features()
        self.feature_table['Max Autocorrelation'] = max_autocorr
        self.feature_table['Autocorr Lag'] = autocorr_lag
        self.feature_table['Autocorr Decay'] = autocorr_decay
        
        # Wavelet energy features
        low_freq_energy, mid_freq_energy, high_freq_energy = self.wavelet_energy_features()
        self.feature_table['Low Freq Energy'] = low_freq_energy
        self.feature_table['Mid Freq Energy'] = mid_freq_energy
        self.feature_table['High Freq Energy'] = high_freq_energy
        
        # Compute energy ratios for additional insights
        total_energy = low_freq_energy + mid_freq_energy + high_freq_energy + 1e-10
        self.feature_table['Low Freq Ratio'] = low_freq_energy / total_energy
        self.feature_table['High Freq Ratio'] = high_freq_energy / total_energy
        
        # Signal complexity features
        fractal_dim, approx_entropy = self.signal_complexity_features()
        self.feature_table['Fractal Dimension'] = fractal_dim
        self.feature_table['Approximate Entropy'] = approx_entropy
    
    def feature_full_monty(self, label, destination: Optional[str] = None) -> pd.DataFrame:
        """
        Extract all features and assemble the complete feature table.

        Parameters:
            label: Label for the feature table
            destination: Optional directory to save the feature table

        Returns:
            pd.DataFrame: Complete feature table with all extracted features
        """

        self._info("Extracting complete feature set...")

        self.initialize_feature_table()
        self.magnitude_features()
        self.phase_features()
        self.spectral_features()
        self.depth_segmented_features()
        self.attenuation_features()
        # self.advanced_dsp_features()

        self.feature_table['Label'] = label

        if destination is not None:
            if not os.path.exists(destination):
                os.makedirs(destination)
            save_path = os.path.join(destination, 'features.csv')
            self.feature_table.to_csv(save_path, index=False)
            self._info(f"Feature table saved to {save_path}")

        self._info(f"Feature extraction complete. Shape: {self.feature_table.shape}")
        return self.feature_table
    

def mutual_info_minimize_features(feature_table: pd.DataFrame, top_n: int = 10) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """
    Select features using mutual information with improved efficiency.

    Parameters:
        feature_table: DataFrame containing features and labels
        top_n: Number of top features to select

    Returns:
        Tuple of (selected_features_df, mutual_info_scores_df)
    """

    print(f"[INFO] Selecting top {top_n} features using mutual information...")
    
    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    labels = feature_table['Label'].values

    # Compute mutual information
    mi_scores = mutual_info_regression(feature_array, labels)
    top_feature_indices = np.argsort(mi_scores)[-top_n:][::-1]  # Descending order
    
    selected_features = [feature_names[i] for i in top_feature_indices]
    selected_mi_scores = [float(mi_scores[i]) for i in top_feature_indices]

    # Create results
    best_features_array = feature_array[:, top_feature_indices]
    df_best = pd.DataFrame(best_features_array, columns=selected_features)
    df_best['Label'] = labels

    mi_scores_df = pd.DataFrame({
        'Feature': selected_features,
        'Mutual Information': selected_mi_scores
    })

    return df_best, mi_scores_df

def correlation_minimize_features(feature_table: pd.DataFrame, top_n: int = 10) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """
    Select features using correlation analysis with improved efficiency.

    Parameters:
        feature_table: DataFrame containing features and labels
        top_n: Number of top features to select

    Returns:
        Tuple of (selected_features_df, correlation_scores_df)
    """

    print(f"[INFO] Selecting top {top_n} features using correlation analysis...")
    
    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    labels = feature_table['Label'].values

    # Compute correlations efficiently
    correlations = np.array([np.corrcoef(labels, feature_array[:, i])[0, 1] 
                           for i in range(feature_array.shape[1])])
    
    # Handle NaN values
    correlations = np.nan_to_num(correlations, nan=0.0)
    
    top_feature_indices = np.argsort(np.abs(correlations))[-top_n:][::-1]
    selected_features = [feature_names[i] for i in top_feature_indices]
    selected_correlations = [float(correlations[i]) for i in top_feature_indices]

    # Create results
    best_features_array = feature_array[:, top_feature_indices]
    df_best = pd.DataFrame(best_features_array, columns=selected_features)
    df_best['Label'] = labels

    corr_scores_df = pd.DataFrame({
        'Feature': selected_features,
        'Correlation': selected_correlations
    })

    return df_best, corr_scores_df

def save_feature_table(feature_table: pd.DataFrame, destination: str, 
                      feature_file_name: str = 'features_selected.csv') -> None:
    """
    Save feature table to CSV with improved error handling.

    Parameters:
        feature_table: DataFrame to save
        destination: Directory to save the file
        feature_file_name: Name of the output file
    """

    try:
        if not os.path.exists(destination):
            os.makedirs(destination)
            
        save_path = os.path.join(destination, feature_file_name)
        feature_table.to_csv(save_path, index=False)
        print(f"[INFO] Feature table saved to {save_path}")
        print(f"[INFO] Saved {feature_table.shape[0]} samples with {feature_table.shape[1]-1} features")
        
    except Exception as e:
        print(f"[ERROR] Failed to save feature table: {e}")

def load_feature_table(directory: str, feature_file_name: str = 'features.csv') -> Tuple[pd.DataFrame, np.ndarray, list, np.ndarray]:
    """
    Load feature table from CSV with improved error handling.

    Parameters:
        directory: Directory containing the feature file
        feature_file_name: Name of the feature file

    Returns:
        Tuple of (feature_table, feature_array, feature_names, labels)
    """

    try:
        file_path = os.path.join(directory, feature_file_name)
        
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Feature file not found: {file_path}")
            
        feature_table = pd.read_csv(file_path)
        
        if 'Label' not in feature_table.columns:
            raise ValueError("Feature table must contain a 'Label' column")

        feature_array = feature_table.drop(columns=['Label']).values
        feature_names = feature_table.drop(columns=['Label']).columns.tolist()
        labels = feature_table['Label'].values

        print(f"[INFO] Loaded feature table: {feature_table.shape[0]} samples, {len(feature_names)} features")
        return feature_table, feature_array, feature_names, labels
        
    except Exception as e:
        print(f"[ERROR] Failed to load feature table: {e}")
        raise