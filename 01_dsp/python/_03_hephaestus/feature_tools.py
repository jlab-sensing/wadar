# Manual feature engineering tools for things such as:
# - Peak Amplitude
# - Peak Shape
# - Signal Entropy

import numpy as np
from scipy.stats import skew, kurtosis
from scipy.signal import find_peaks
import os

class FeatureTools:
    def __init__(self, X, soil_index=200):
        self.X = X
        self.soil_index = soil_index

    def _get_peak_signal(self):
        """
        Extracts the peak signal from the data based on the soil index. The peak 
        signal is defined as the maximum absolute value within a 100-sample window
        starting from the soil index, which is the minimum index of the soil layer.

        TODO: Once a proper mount is in place, we will no longer hardcode a minimum
        because we know the exact soil index.
        """
        peak_idxs = np.argmax(np.abs(self.X[:, self.soil_index:self.soil_index + 100, :]), axis=1)
        peak_idxs = np.median(peak_idxs, axis=1).astype(int) + self.soil_index
        signal = np.abs(self.X[np.arange(self.X.shape[0]), peak_idxs, :])  # shape: (N, C)
        return signal, peak_idxs

    def peak_amplitude(self):
        """
        The peak amplitude correlates with the maximum signal strength. In the
        context of soil, this peak amplitude can be interpreted as the reflectivity
        of the soil layer, which is correlated with soil permitivity, which in turn
        is correlated with soil compaction.
        """
        peak = np.max(np.abs(self.X[:, self.soil_index:self.soil_index + 100, :]), axis=1)
        return np.mean(peak, axis=1)

    def signal_variance(self):
        """
        Computes the variance of the signal. Signal variance is a measure of the
        dispersion of the signal values around their mean. In the context of soil
        analysis, a higher variance may indicate more variability in the soil's
        properties, which may correlate with factors such as soil compaction.
        """
        signal, _ = self._get_peak_signal()
        return np.var(signal, axis=1)

    def signal_entropy(self):
        """
        Computes the entropy of the signal. Signal entropy is a measure of the
        uncertainty or randomness in the signal values. In the context of soil
        analysis, higher entropy may indicate a more complex or heterogeneous soil
        structure, which can be associated with factors such as soil composition.
        """
        signal, _ = self._get_peak_signal()
        entropy = np.zeros(signal.shape[0])
        for i in range(signal.shape[0]):
            normed = signal[i] / (np.sum(signal[i]) + 1e-10)
            entropy[i] = -np.sum(normed * np.log(normed + 1e-10))
        return entropy

    def signal_energy(self):
        """
        Computes the total energy of the signal, defined as the sum of squared 
        amplitudes. Signal energy represents the overall power reflected from 
        the soil layer. More compact soil may yield higher energy due to stronger 
        reflections caused by tighter packing and reduced attenuation.
        """
        signal, _ = self._get_peak_signal()
        return np.sum(signal**2, axis=1)

    def peak_delay(self):
        """
        Computes the time delay of the peak reflection relative to the expected 
        soil index. A smaller delay implies an earlier reflection, which may 
        indicate shallower or more compacted soil. This feature can help estimate 
        changes in dielectric properties that affect wave propagation speed.
        """
        _, peak_idxs = self._get_peak_signal()
        return peak_idxs - self.soil_index

    def peak_width_fwhm(self):
        """
        Computes the Full Width at Half Maximum (FWHM) of the reflected signal 
        around the soil region. A lower FWHM implies a sharper, more distinct 
        reflection, which may hint at a more compacted or uniform soil layer.
        """

        N, T, C = self.X.shape
        fwhm_list = np.zeros(N)
        for i in range(N):
            region = np.abs(self.X[i, self.soil_index:self.soil_index + 100, :])        # Get magnitude of waveform averaged across channels
            waveform = np.mean(region, axis=1)
            peak_val = np.max(waveform)                                                 # Find the peak value and compute FWHM
            half_max = peak_val / 2.0
            indices = np.where(waveform >= half_max)[0]                                 # Indices where waveform crosses half max
            if len(indices) >= 2:
                fwhm_list[i] = indices[-1] - indices[0]
            else:
                fwhm_list[i] = 0

        return fwhm_list

    def signal_skewness_kurtosis(self):
        """
        Computes the skewness and kurtosis of the signal. 
        - Skewness captures asymmetry of the signal distribution.
        - Kurtosis measures the 'peakedness' or presence of heavy tails.
        These higher-order statistics help characterize the shape of the 
        reflected waveform, offering insight into the soil surface texture 
        or subsurface structure.
        """
        signal, _ = self._get_peak_signal()
        skewness = np.zeros(signal.shape[0])
        kurt_vals = np.zeros(signal.shape[0])
        for i in range(signal.shape[0]):
            skewness[i] = skew(signal[i])
            kurt_vals[i] = kurtosis(signal[i])
        return skewness, kurt_vals

    def spectral_centroid_bandwidth(self):
        """
        Computes the spectral centroid and bandwidth of the signal using the FFT.
        - The centroid indicates where most of the signal’s energy is concentrated 
          in frequency space.
        - Bandwidth quantifies the spread of frequencies.
        These frequency-domain features can be linked to subsurface scattering 
        and material transitions, both influenced by compaction. I don't fully
        understand how these relate to soil compaction, but I've seen them
        used in permittivity estimation in literature.
        """
        signal, _ = self._get_peak_signal()
        centroid = np.zeros(signal.shape[0])
        bandwidth = np.zeros(signal.shape[0])
        for i in range(signal.shape[0]):
            spec = np.fft.fft(signal[i])
            mag = np.abs(spec[:len(spec)//2])
            freqs = np.fft.fftfreq(len(signal[i]))[:len(mag)]
            mag_sum = np.sum(mag) + 1e-10
            centroid[i] = np.sum(freqs * mag) / mag_sum
            bandwidth[i] = np.sqrt(np.sum(((freqs - centroid[i])**2) * mag) / mag_sum)
        return centroid, bandwidth

    def entropy_to_energy_ratio(self):
        """
        Computes the ratio of signal entropy to energy. 
        This hybrid feature captures the balance between signal complexity 
        and intensity. High values may indicate weak but noisy returns, while 
        low values suggest strong and structured reflections — both of which 
        may relate to changes in soil density.
        """
        entropy = self.signal_entropy()
        energy = self.signal_energy()
        return entropy / (energy + 1e-10)

    def peak_to_entropy_ratio(self):
        """
        Computes the ratio of peak amplitude to entropy. 
        A high ratio reflects a dominant, sharp reflection (e.g., from compacted 
        soil), while a lower ratio indicates a flatter or noisier return. 
        This is a useful composite measure of signal sharpness and structure.
        """
        peak = self.peak_amplitude()
        entropy = self.signal_entropy()
        return peak / (entropy + 1e-10)
    
    def signal_attenuation(self):
        # TODO: Implement this.

    def save_features(self, dataset_dir, normalize=True):
        """
        Saves the computed features to a file.
        """

        self.feature_names = [
            'peak_amplitude',
            'signal_variance',
            'signal_entropy',
            'signal_energy',
            'peak_delay',
            'peak_width_fwhm',
            'signal_skewness',
            'signal_kurtosis',
            'spectral_centroid',
            'spectral_bandwidth',
            'entropy_to_energy_ratio',
            'peak_to_entropy_ratio'
        ]

        if normalize:
            features = {
                'peak_amplitude': self.peak_amplitude() / np.max(self.peak_amplitude()),
                'signal_variance': self.signal_variance() / np.max(self.signal_variance()),
                'signal_entropy': self.signal_entropy() / np.max(self.signal_entropy()),
                'signal_energy': self.signal_energy() / np.max(self.signal_energy()),
                'peak_delay': self.peak_delay() / np.max(np.abs(self.peak_delay())),
                'peak_width_fwhm': self.peak_width_fwhm() / np.max(self.peak_width_fwhm()),
                'signal_skewness': self.signal_skewness_kurtosis()[0] / np.max(np.abs(self.signal_skewness_kurtosis()[0])),
                'signal_kurtosis': self.signal_skewness_kurtosis()[1] / np.max(np.abs(self.signal_skewness_kurtosis()[1])),
                'spectral_centroid': self.spectral_centroid_bandwidth()[0] / np.max(np.abs(self.spectral_centroid_bandwidth()[0])),
                'spectral_bandwidth': self.spectral_centroid_bandwidth()[1] / np.max(np.abs(self.spectral_centroid_bandwidth()[1])),
                'entropy_to_energy_ratio': self.entropy_to_energy_ratio() / np.max(np.abs(self.entropy_to_energy_ratio())),
                'peak_to_entropy_ratio': self.peak_to_entropy_ratio() / np.max(np.abs(self.peak_to_entropy_ratio()))
            }
        else:
            features = {
                'peak_amplitude': self.peak_amplitude(),
                'signal_variance': self.signal_variance(),
                'signal_entropy': self.signal_entropy(),
                'signal_energy': self.signal_energy(),
                'peak_delay': self.peak_delay(),
                'peak_width_fwhm': self.peak_width_fwhm(),
                'signal_skewness': self.signal_skewness_kurtosis()[0],
                'signal_kurtosis': self.signal_skewness_kurtosis()[1],
                'spectral_centroid': self.spectral_centroid_bandwidth()[0],
                'spectral_bandwidth': self.spectral_centroid_bandwidth()[1],
                'entropy_to_energy_ratio': self.entropy_to_energy_ratio(),
                'peak_to_entropy_ratio': self.peak_to_entropy_ratio()
            }
    
        fileName = f"{dataset_dir}/features.npz"
        if os.path.exists(fileName):
            print(f"Warning: {fileName} already exists. Overwriting.")
        np.savez(fileName, **features)