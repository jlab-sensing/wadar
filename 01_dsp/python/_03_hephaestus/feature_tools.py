# Manual feature engineering tools for things such as:
# - Peak Amplitude
# - Peak Shape
# - Signal Entropy

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import skew, kurtosis
from scipy.signal import find_peaks
import os
from scipy.signal import peak_widths
import pandas as pd

class FeatureTools:
    def __init__(self, X):
        self.X = X

    def _average_frame(self, scan_idx=0):
        """
        Just so I can easily decide whether to use median or mean.
        """
        return np.abs(np.median(self.X[scan_idx, :, :], axis=1))
    

    def _get_peak_idx(self, scan_idx = 0):
        """
        Gets the two biggest peaks. The first one, in the near-field of the
        radar, is technically the effect of the antenna coupling, but it seems
        to change with soil compaction. The second one is the actual
        reflection from the soil layer.
        """

        scan_mean = self._average_frame(scan_idx)

        peak_idxs = find_peaks(scan_mean)

        if len(peak_idxs) < 2:
            return peak_idxs
        else:
            sorted_peaks = np.argsort(scan_mean[peak_idxs[0]])[-2:]
            return peak_idxs[0][sorted_peaks][::-1] 
        
    def peak_amplitude(self):

        peak_amplitudes = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            peak_idxs = self._get_peak_idx(i)
            signal = self._average_frame(i)
            peak_amplitudes[i, 0] = signal[peak_idxs[0]]
            peak_amplitudes[i, 1] = signal[peak_idxs[1]]
        
        return peak_amplitudes

    def _get_signal_variance(self, scan_idx=0, spec_idx=0):
        """
        Signal variance at scan_idx at the specified index.
        """
        signal = np.abs(self.X[scan_idx, :, :])
        signal = signal[spec_idx]
        return np.var(signal)
    
    def signal_variance(self, spec_idx=0):
        variance = np.zeros(self.X.shape[0])
        for i in range(self.X.shape[0]):
            variance[i] = self._get_signal_variance(i, spec_idx)
        return variance
    
    def peak_variance(self):
        peak_idxs = self._get_peak_idx()
        return [
            self.signal_variance(peak_idxs[0]),
            self.signal_variance(peak_idxs[1])
        ]
    
    def _get_amplitude2variance_ratio(self, scan_idx=0, spec_idx=0):
        signal = self._average_frame(scan_idx)
        amplitude = signal[spec_idx]
        variance = self._get_signal_variance(scan_idx, spec_idx)

        return amplitude / (variance + 1e-10)
    
    def peak_amplitude2variance_ratio(self):
        ratios = np.zeros((2, self.X.shape[0]))
        for i in range(self.X.shape[0]):
            peak_idxs = self._get_peak_idx(i)
            ratios[0, i] = self._get_amplitude2variance_ratio(i, peak_idxs[0])
            ratios[1, i] = self._get_amplitude2variance_ratio(i, peak_idxs[1])
        return ratios
    
    def _get_signal_entropy(self, scan_idx=0, spec_idx=0):
        """
        Signal entropy at scan_idx at the specified index.
        """
        signal = np.abs(self.X[scan_idx, :, :])
        signal = signal[spec_idx]
        normed = signal / (np.sum(signal) + 1e-10)
        return -np.sum(normed * np.log(normed + 1e-10))
    
    def signal_entropy(self, spec_idx=0):
        entropy = np.zeros(self.X.shape[0])
        for i in range(self.X.shape[0]):
            entropy[i] = self._get_signal_entropy(i, spec_idx)
        return entropy
    
    def peak_entropy(self):
        peak_idxs = self._get_peak_idx()
        return [
            self.signal_entropy(peak_idxs[0]),
            self.signal_entropy(peak_idxs[1])
        ]
    
    def _get_amplitude2entropy_ratio(self, scan_idx=0, spec_idx=0):
        signal = self._average_frame(scan_idx)
        amplitude = signal[spec_idx]
        entropy = self._get_signal_entropy(scan_idx, spec_idx)

        return amplitude / (entropy + 1e-10)
    
    def peak_amplitude2entropy_ratio(self):
        ratios = np.zeros((2, self.X.shape[0]))
        for i in range(self.X.shape[0]):
            peak_idxs = self._get_peak_idx(i)
            ratios[0, i] = self._get_amplitude2entropy_ratio(i, peak_idxs[0])
            ratios[1, i] = self._get_amplitude2entropy_ratio(i, peak_idxs[1])
        return ratios
    
    def peak_delay(self):
        """
        A bunch of delays
        """
        delays = np.zeros((3, self.X.shape[0]))
        for i in range(self.X.shape[0]):
            peak_idxs = self._get_peak_idx(i)
            delays[0, i] = peak_idxs[0]                 # distance to first peak
            delays[1, i] = peak_idxs[1]                 # distance to second peak
            delays[2, i] = peak_idxs[1] - peak_idxs[0]  # distance between peaks
        
        return delays
    
    def _get_peak_width(self, scan_idx=0):
        signal = self._average_frame(scan_idx)
        peaks = self._get_peak_idx(scan_idx)
        peak1, peak2 = peaks[0], peaks[1]
        width1 = peak_widths(signal, [peak1], rel_height=0.5)[0][0]
        width2 = peak_widths(signal, [peak2], rel_height=0.5)[0][0]
        return width1, width2
    
    def peak_width(self):
        widths = np.zeros((2, self.X.shape[0]))
        for i in range(self.X.shape[0]):
            widths[0, i], widths[1, i] = self._get_peak_width(i)
        return widths
    
    def _get_peak_shape_stats(self, scan_idx=0, window=5):
        peak_idxs = self._get_peak_idx(scan_idx)
        signal = self._average_frame(scan_idx)
        start1 = max(0, peak_idxs[0] - window)
        end1 = min(len(signal), peak_idxs[0] + window)
        window_signal1 = signal[start1:end1]
        start2 = max(0, peak_idxs[1] - window)
        end2 = min(len(signal), peak_idxs[1] + window)
        window_signal2 = signal[start2:end2]
        return skew(window_signal1), kurtosis(window_signal1), skew(window_signal2), kurtosis(window_signal2)

    def peak_shape_stats(self):
        skewness = np.zeros((2, self.X.shape[0]))
        kurtosis_vals = np.zeros((2, self.X.shape[0]))
        for i in range(self.X.shape[0]):
            skew1, kurt1, skew2, kurt2 = self._get_peak_shape_stats(i)
            skewness[0, i] = skew1
            skewness[1, i] = skew2
            kurtosis_vals[0, i] = kurt1
            kurtosis_vals[1, i] = kurt2
        return skewness, kurtosis_vals
    
    def _get_signal_energy(self, scan_idx=0, spec_idx=0):
        signal = np.abs(self.X[scan_idx, :, :])[spec_idx]
        return np.sum(signal ** 2)
    
    def signal_energy(self, spec_idx=0):
        energy = np.zeros(self.X.shape[0])
        for i in range(self.X.shape[0]):
            energy[i] = self._get_signal_energy(i, spec_idx)
        return energy
    
    def peak_signal_energy(self):
        peak_idxs = self._get_peak_idx()
        return [
            self.signal_energy(peak_idxs[0]),
            self.signal_energy(peak_idxs[1])
        ]
    
    def _get_decay_rate(self, scan_idx=0, points_after=10):
        signal = self._average_frame(scan_idx)
        peak_idx = self._get_peak_idx(scan_idx)

        start1 = peak_idx[0]
        end1 = min(len(signal), peak_idx[0] + points_after)
        x1 = np.arange(start1, end1)
        y1 = signal[start1:end1]
        slope1, _ = np.polyfit(x1, y1, 1)

        start2 = peak_idx[1]
        end2 = min(len(signal), peak_idx[1] + points_after)
        x2 = np.arange(start2, end2)
        y2 = signal[start2:end2]
        slope2, _ = np.polyfit(x2, y2, 1)

        return slope1, slope2
    
    def decay_rate(self):
        slopes = np.zeros((2, self.X.shape[0]))
        for i in range(self.X.shape[0]):
            slopes[0, i], slopes[1, i] = self._get_decay_rate(i)
        return slopes
    
    def _get_ascend_rate(self, scan_idx=0, points_before=10):
        signal = self._average_frame(scan_idx)
        peak_idx = self._get_peak_idx(scan_idx)

        start1 = max(0, peak_idx[0] - points_before)
        end1 = peak_idx[0]
        x1 = np.arange(start1, end1)
        y1 = signal[start1:end1]
        slope1, _ = np.polyfit(x1, y1, 1)

        start2 = max(0, peak_idx[1] - points_before)
        end2 = peak_idx[1]
        x2 = np.arange(start2, end2)
        y2 = signal[start2:end2]
        slope2, _ = np.polyfit(x2, y2, 1)

        return slope1, slope2
    
    def ascend_rate(self):
        slopes = np.zeros((2, self.X.shape[0]))
        for i in range(self.X.shape[0]):
            slopes[0, i], slopes[1, i] = self._get_ascend_rate(i)
        return slopes

    def _get_phase_variance(self, scan_idx=0, spec_idx=0):
        signal = self.X[scan_idx, :, :][spec_idx]
        phase = np.angle(signal)
        return np.var(phase)

    def phase_variance(self, spec_idx=0):
        variance = np.zeros(self.X.shape[0])
        for i in range(self.X.shape[0]):
            variance[i] = self._get_phase_variance(i, spec_idx)
        return variance

    def peak_phase_variance(self):
        peak_idxs = self._get_peak_idx()
        return [
            self.phase_variance(peak_idxs[0]),
            self.phase_variance(peak_idxs[1])
        ]

    def _get_circularity_coefficient(self, scan_idx=0, spec_idx=0):
        signal = self.X[scan_idx, :, :][spec_idx]
        mean_conj_prod = np.mean(signal * signal)
        mean_power = np.mean(np.abs(signal) ** 2)
        return np.abs(mean_conj_prod) / (mean_power + 1e-10)

    def circularity_coefficient(self, spec_idx=0):
        coeff = np.zeros(self.X.shape[0])
        for i in range(self.X.shape[0]):
            coeff[i] = self._get_circularity_coefficient(i, spec_idx)
        return coeff

    def peak_circularity_coefficient(self):
        peak_idxs = self._get_peak_idx()
        return [
            self.circularity_coefficient(peak_idxs[0]),
            self.circularity_coefficient(peak_idxs[1])
        ]

    def _get_phase_jitter(self, scan_idx=0, spec_idx=0):
        signal = self.X[scan_idx, :, :][spec_idx]
        phase = np.unwrap(np.angle(signal))
        return np.var(np.diff(phase))

    def phase_jitter(self, spec_idx=0):
        jitter = np.zeros(self.X.shape[0])
        for i in range(self.X.shape[0]):
            jitter[i] = self._get_phase_jitter(i, spec_idx)
        return jitter

    def peak_phase_jitter(self):
        peak_idxs = self._get_peak_idx()
        return [
            self.phase_jitter(peak_idxs[0]),
            self.phase_jitter(peak_idxs[1])
        ]

