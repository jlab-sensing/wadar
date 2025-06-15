# Manual feature engineering tools for things such as:
# - Peak Amplitude
# - Peak Shape
# - Signal Entropy

import numpy as np

class FeatureTools:
    def __init__(self, X, soil_index=200):
        self.X = X
        self.soil_index = soil_index # This is where the soil is approximately located in the frame. We do this manually until we have a better way to determine it.

    def peak_amplitude(self):
        """
        Calculate the peak amplitude of the signal.
        """
        peak_amplitude = np.max(np.abs(self.X[:, self.soil_index:self.soil_index + 100, :]), axis=1)
        peak_amplitude = np.mean(peak_amplitude, axis=1)
        return peak_amplitude
    
    def signal_variance(self):
        """
        Calculate the signal variance across the soil region.
        """
        peak_idxs = np.argmax(np.abs(self.X[:, self.soil_index:self.soil_index + 100, :]), axis=1)
        peak_idxs = np.median(peak_idxs, axis=1).astype(int) + self.soil_index
        signal = self.X[:, peak_idxs, :]
        signal = np.abs(signal)  
        signal_variance = np.zeros(signal.shape[0], dtype=np.float64)
        for i in range(signal.shape[0]):
            signal_normed = signal[i, :] / (np.linalg.norm(signal[i, :])) # L2 norm because otherwise the variance of higher amplitude signals will be higher
            signal_variance[i] = np.var(signal_normed)
        return signal_variance

    def signal_entropy(self):
        """
        Calculate the normalized signal entropy across the soil region. TODO: double check this, it's from ChatGPT.
        """
        peak_idxs = np.argmax(np.abs(self.X[:, self.soil_index:self.soil_index + 100, :]), axis=1)
        peak_idxs = np.median(peak_idxs, axis=1).astype(int) + self.soil_index
        signal = np.abs(self.X[np.arange(self.X.shape[0]), peak_idxs, :])  

        signal_entropy = np.zeros(signal.shape[0], dtype=np.float64)
        for i in range(signal.shape[0]):
            signal_normed = signal[i, :] / (np.sum(signal[i, :]) + 1e-10)  # Normalize to sum=1 for probability dist
            signal_entropy[i] = -np.sum(signal_normed * np.log(signal_normed + 1e-10))  # Shannon entropy

        return signal_entropy