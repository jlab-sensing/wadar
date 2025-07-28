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
    feature_names = [name.replace('_', ' ').title() for name in feature_names]
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
    feature_names = [name.replace('_', ' ').title() for name in feature_names]
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
        feature_names = [name.replace('_', ' ').title() for name in feature_names]
        labels = feature_table['Label'].values

        print(f"[INFO] Loaded feature table: {feature_table.shape[0]} samples, {len(feature_names)} features")
        return feature_table, feature_array, feature_names, labels
        
    except Exception as e:
        print(f"[ERROR] Failed to load feature table: {e}")
        raise