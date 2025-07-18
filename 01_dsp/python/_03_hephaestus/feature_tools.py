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
from sklearn.linear_model import Lasso, LassoCV
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from scipy.stats import entropy
from sklearn.feature_selection import mutual_info_regression

class FeatureTools:
    def __init__(self, X):
        self.X = X

    def _segments(self):
        """
        So I can determine the segments of the signal that I want to analyze.
        """

        return [0, 250, 512]

    def _average_frame(self):
        """
        Just so I can easily decide whether to use median or mean.
        """

        return np.median(self.X, axis=2)
    
    def _get_peak_idx(self):
        """
        Returns the two largest peaks in the signal for all scans.
        """
        peak_idxs = np.zeros((self.X.shape[0], 2), dtype=int)
        average_signals = np.abs(self._average_frame())
        for i in range(self.X.shape[0]):
            signal = average_signals[i]
            peaks, _ = find_peaks(signal, distance=5)
            top2 = np.argsort(signal[peaks])[-2:]  # Get indices of the two largest peaks
            peak_idxs[i] = peaks[top2]

        return peak_idxs

    def peak_amplitude(self):
        """
        Amplitude of the two peaks for all scans.
        """

        peak_idxs = self._get_peak_idx()
        signal = np.abs(self._average_frame())

        peak_amplitudes = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            peak_amplitudes[i, 0] = signal[i, peak_idxs[i, 0]]
            peak_amplitudes[i, 1] = signal[i, peak_idxs[i, 1]]
        return peak_amplitudes

    def _get_signal_variance(self, idx):
        """
        Computes the variance of the signal at the specified index.
        """

        signal = np.abs(self.X[idx, :, :])
        return np.var(signal, axis=1)
        
    def peak_variance(self):
        """
        Variance of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        variance = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            signal_var = self._get_signal_variance(i)
            variance[i, 0] = signal_var[peak_idxs[i, 0]]
            variance[i, 1] = signal_var[peak_idxs[i, 1]]
        return variance

    def _get_signal_entropy(self, idx):
        """
        Computes the entropy of the signal at the specified index.
        Signal entropy is a measure of the uncertainty in the signal.
        """
        signal = np.abs(self.X[idx, :, :])
        entropy = np.zeros(signal.shape[0])
        for i in range(signal.shape[0]):
            hist, _ = np.histogram(signal[i, :], bins=30, density=True)
            hist = hist[hist > 0]
            entropy[i] = -np.sum(hist * np.log(hist + 1e-10))
        return entropy

    def peak_entropy(self):
        """
        Entropy of the two peaks.
        """
        peak_idxs = self._get_peak_idx()
        entropy = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            signal_entropy = self._get_signal_entropy(i)
            entropy[i, 0] = signal_entropy[peak_idxs[i, 0]]
            entropy[i, 1] = signal_entropy[peak_idxs[i, 1]]
        return entropy
    
    def peak_delay(self):
        """
        A bunch of delays. Index 0 is the distance to when the antenna
        coupling peak occurs, index 1 is the distance to the second peak
        which is the actual soil reflection, and index 2 is the relative
        difference between the two peaks.
        """

        peak_idxs = self._get_peak_idx() 
        delays = np.zeros((self.X.shape[0], 3))
        delays[:, 0] = peak_idxs[:, 0]
        delays[:, 1] = peak_idxs[:, 1]
        delays[:, 2] = peak_idxs[:, 1] - peak_idxs[:, 0]
        return delays

    def peak_width(self):
        """
        Width of the two peaks at each scan.
        """
        peak_idxs = self._get_peak_idx()
        signal = np.abs(self._average_frame())
        widths = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            widths[i, 0] = peak_widths(signal[i], [peak_idxs[i, 0]])[0][0]
            widths[i, 1] = peak_widths(signal[i], [peak_idxs[i, 1]])[0][0]
        return widths

    def peak_shape_stats(self):
        """
        Skewness and kurtosis of the two peaks at each scan.
        Skewness measures the asymmetry of the signal,
        while kurtosis measures the peakedbess of the signal.
        """

        skewness = np.zeros((self.X.shape[0], 2))
        kurt = np.zeros((self.X.shape[0], 2))
        peak_idxs = self._get_peak_idx()
        signal = np.abs(self._average_frame())
        for i in range(self.X.shape[0]):
            skewness[i, 0] = skew(signal[i, :peak_idxs[i, 0]])
            skewness[i, 1] = skew(signal[i, peak_idxs[i, 1]:])
            kurt[i, 0] = kurtosis(signal[i, :peak_idxs[i, 0]])
            kurt[i, 1] = kurtosis(signal[i, peak_idxs[i, 1]:])
        return skewness, kurt
    
    def _get_signal_energy(self, scan_idx=0, spec_idx=0):
        """
        Computes the energy of the signal at scan_idx at the specified index.
        Not that different from amplitude but accounts for the entire signal
        taken over slow time.
        """

        signal = np.abs(self.X[scan_idx, :, :])[spec_idx]
        return np.sum(signal ** 2)
    
    def peak_signal_energy(self):
        """
        Computes the energy of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        energy = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            energy[i, 0] = self._get_signal_energy(i, peak_idxs[i, 0])
            energy[i, 1] = self._get_signal_energy(i, peak_idxs[i, 1])
        return energy
    
    def decay_rate(self, decay_points=10):
        """
        Computes the decay rate of the two peaks for all scans.
        """

        slopes = np.zeros((self.X.shape[0], 2))
        signal = np.abs(self._average_frame())

        peak_idxs = self._get_peak_idx()

        # peak 1
        slopes = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            start1 = peak_idxs[i, 0]
            end1 = min(signal.shape[1], peak_idxs[i, 0] + decay_points)
            x1 = np.arange(start1, end1)
            y1 = signal[i, start1:end1]
            slopes[i, 0], _ = np.polyfit(x1, y1, 1)

        # peak 2
        for i in range(self.X.shape[0]):
            start2 = peak_idxs[i, 1]
            end2 = min(signal.shape[1], peak_idxs[i, 1] + decay_points)
            x2 = np.arange(start2, end2)
            y2 = signal[i, start2:end2]
            slopes[i, 1], _ = np.polyfit(x2, y2, 1)

        return slopes

    def ascend_rate(self):
        """
        Computes the ascend rate of the two peaks for all scans.
        """

        slopes = np.zeros((self.X.shape[0], 2))
        signal = np.abs(self._average_frame())

        peak_idxs = self._get_peak_idx()

        # peak 1
        for i in range(self.X.shape[0]):
            start1 = max(0, peak_idxs[i, 0] - 10)
            end1 = peak_idxs[i, 0]
            x1 = np.arange(start1, end1)
            y1 = signal[i, start1:end1]
            slopes[i, 0], _ = np.polyfit(x1, y1, 1)

        # peak 2
        for i in range(self.X.shape[0]):
            start2 = max(0, peak_idxs[i, 1] - 10)
            end2 = peak_idxs[i, 1]
            x2 = np.arange(start2, end2)
            y2 = signal[i, start2:end2]
            slopes[i, 1], _ = np.polyfit(x2, y2, 1)

        return slopes
    
    def peak_phase(self):
        """
        Computes the phase of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        signal = np.angle(self._average_frame())
        peak_phases = np.zeros((self.X.shape[0], 2))

        for i in range(self.X.shape[0]):
            peak_phases[i, 0] = signal[i, peak_idxs[i, 0]]
            peak_phases[i, 1] = signal[i, peak_idxs[i, 1]]
        return peak_phases
            
    def _get_phase_variance(self, idx):
        """
        Computes the variance of the phase of the signal at scan_idx.
        This is because the stochastic nature of the signal reflecting
        on more porous soil may affect the phase of the signal.
        """

        signal = np.angle(self.X[idx, :, :])
        return np.var(signal, axis=1)

    def peak_phase_variance(self):
        """
        Computes the phase variance of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        phase_variance = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            signal_phase_var = self._get_phase_variance(i)
            phase_variance[i, 0] = signal_phase_var[peak_idxs[i, 0]]
            phase_variance[i, 1] = signal_phase_var[peak_idxs[i, 1]]
        return phase_variance

    def _get_circularity_coefficient(self, idx=0):
        """
        Computes the circularity coefficient of the signal at idx.
        The circularity coefficient describes the statistical
        scattering behavior of the signal. 
        """

        signal = self.X[idx, :, :]

        coeffs = np.zeros(signal.shape[0])
        for i in range(signal.shape[0]):
            cur_signal = signal[i, :]
            mean_conj_prod = np.mean(cur_signal * cur_signal)
            mean_power = np.mean(np.abs(cur_signal) ** 2)
            coeffs[i] = np.abs(mean_conj_prod) / (mean_power + 1e-10)
        return coeffs


    def peak_circularity_coefficient(self):
        """
        Computes the circularity coefficient of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        coeffs = np.zeros((self.X.shape[0], 2))

        for i in range(self.X.shape[0]):
            circ_coeffs = self._get_circularity_coefficient(i)
            coeffs[i, 0] = circ_coeffs[peak_idxs[i, 0]]
            coeffs[i, 1] = circ_coeffs[peak_idxs[i, 1]]
        return coeffs

    def _get_phase_jitter(self, scan_idx=0, spec_idx=0):
        """
        Computes the phase jitter of the signal at scan_idx.
        Phase jitter is the variance of the phase difference
        between consecutive samples in the signal.
        """

        signal = self.X[scan_idx, :, :][spec_idx]
        phase = np.unwrap(np.angle(signal))
        return np.var(np.diff(phase))

    def peak_phase_jitter(self):
        """
        Computes the phase jitter of the two peaks for all scans.
        """

        peak_idxs = self._get_peak_idx()
        phase_jitter = np.zeros((self.X.shape[0], 2))
        for i in range(self.X.shape[0]):
            phase_jitter[i, 0] = self._get_phase_jitter(i, peak_idxs[i, 0])
            phase_jitter[i, 1] = self._get_phase_jitter(i, peak_idxs[i, 1])
        return phase_jitter
    
    # TODO: FFT and some time domain features. Maybe correlation between peaks?
    #       There are advanced radar clutter analysis techniques.
    #       Persistence, but that might be the same as variance.
    #       Definitely missing phase stuff, so prioritize that.

    def initialize_feature_table(self):
        """
        Assembles a feature table with all the features extracted.
        """

        self.feature_table = pd.DataFrame({})
    
    def magnitude_features(self):
        """
        Extracts all the features related to the magnitude of the signal.
        """

        peak_amplitude = self.peak_amplitude()
        self.feature_table['Peak Amplitude 1'] = peak_amplitude[:, 0]
        self.feature_table['Peak Amplitude 2'] = peak_amplitude[:, 1]
        peak_variance = self.peak_variance()
        self.feature_table['Peak Variance 1'] = peak_variance[:, 0]
        self.feature_table['Peak Variance 2'] = peak_variance[:, 1]
        peak_entropy = self.peak_entropy()
        self.feature_table['Peak Entropy 1'] = peak_entropy[:, 0]
        self.feature_table['Peak Entropy 2'] = peak_entropy[:, 1]
        amplitude2variance_ratio = peak_amplitude / (peak_variance + 1e-10)
        self.feature_table['Amplitude to Variance Ratio 1'] = amplitude2variance_ratio[:, 0]
        self.feature_table['Amplitude to Variance Ratio 2'] = amplitude2variance_ratio[:, 1]
        amplitude2entropy_ratio = peak_amplitude / (peak_entropy + 1e-10)
        self.feature_table['Amplitude to Entropy Ratio 1'] = amplitude2entropy_ratio[:, 0]
        self.feature_table['Amplitude to Entropy Ratio 2'] = amplitude2entropy_ratio[:, 1]
        peak_delay = self.peak_delay()
        self.feature_table['Peak Delay 1'] = peak_delay[:, 0]
        self.feature_table['Peak Delay 2'] = peak_delay[:, 1]
        self.feature_table['Peak Delay Difference'] = peak_delay[:, 2]
        peak_width = self.peak_width()
        self.feature_table['Peak Width 1'] = peak_width[:, 0]
        self.feature_table['Peak Width 2'] = peak_width[:, 1]
        peak_skew, peak_kurt = self.peak_shape_stats()
        self.feature_table['Peak Skewness 1'] = peak_skew[:, 0]
        self.feature_table['Peak Skewness 2'] = peak_skew[:, 1]
        self.feature_table['Peak Kurtosis 1'] = peak_kurt[:, 0]
        self.feature_table['Peak Kurtosis 2'] = peak_kurt[:, 1]
        peak_energy = self.peak_signal_energy()
        self.feature_table['Peak Energy 1'] = peak_energy[:, 0]
        self.feature_table['Peak Energy 2'] = peak_energy[:, 1]
        decay_rate = self.decay_rate()
        self.feature_table['Decay Rate 1'] = decay_rate[:, 0]
        self.feature_table['Decay Rate 2'] = decay_rate[:, 1]
        ascend_rate = self.ascend_rate()
        self.feature_table['Ascend Rate 1'] = ascend_rate[:, 0]
        self.feature_table['Ascend Rate 2'] = ascend_rate[:, 1]

    def phase_features(self):
        """
        Extracts all the features related to the phase of the signal.
        """

        peak_phase = self.peak_phase()
        self.feature_table['Peak Phase 1'] = peak_phase[:, 0]
        self.feature_table['Peak Phase 2'] = peak_phase[:, 1]
        phase_variance = self.peak_phase_variance()
        self.feature_table['Phase Variance 1'] = phase_variance[:, 0]
        self.feature_table['Phase Variance 2'] = phase_variance[:, 1]
        circularity_coefficient = self.peak_circularity_coefficient()
        self.feature_table['Circularity Coefficient 1'] = circularity_coefficient[:, 0]
        self.feature_table['Circularity Coefficient 2'] = circularity_coefficient[:, 1]
        phase_jitter = self.peak_phase_jitter()
        self.feature_table['Phase Jitter 1'] = phase_jitter[:, 0]
        self.feature_table['Phase Jitter 2'] = phase_jitter[:, 1]
    
    def feature_full_monty(self, label, destination=None):
        """
        Extracts all the features related to the signal.
        This includes both magnitude and phase features.
        """

        self.initialize_feature_table()
        self.magnitude_features()
        self.phase_features()

        self.feature_table['Label'] = label

        if destination is not None:
            if not os.path.exists(destination):
                os.makedirs(destination)
            self.feature_table.to_csv(os.path.join(destination, 'features.csv'), index=False)

        return self.feature_table
    

def mutual_info_minimize_features(feature_table, top_n=10):
    
    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    feature_names = [name.replace('_', ' ').title() for name in feature_names]
    labels = feature_table['Label'].values

    mi = mutual_info_regression(feature_array, labels)
    top_features = np.argsort(mi)[-top_n:][::-1]  # Descending order
    selected_features = [feature_names[i] for i in top_features]
    selected_mi = [float(mi[i]) for i in top_features]

    best_features_array = feature_array[:, top_features]
    df_best = pd.DataFrame(best_features_array, columns=selected_features)
    df_best['Label'] = labels

    # scores
    mi_scores = pd.DataFrame({
        'Feature': selected_features,
        'Mutual Information': selected_mi
    })

    return df_best, mi_scores

def correlation_minimize_features(feature_table, top_n=10):
    
    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    feature_names = [name.replace('_', ' ').title() for name in feature_names]
    labels = feature_table['Label'].values

    corr = np.array([np.corrcoef(labels, feature_array[:, i])[0, 1] for i in range(feature_array.shape[1])])
    top_features = np.argsort(np.abs(corr))[-top_n:][::-1]  
    selected_features = [feature_names[i] for i in top_features]
    selected_corr = [float(corr[i]) for i in top_features]

    best_features_array = feature_array[:, top_features]
    df_best = pd.DataFrame(best_features_array, columns=selected_features)
    df_best['Label'] = labels

    # scores
    corr_scores = pd.DataFrame({
        'Feature': selected_features,
        'Correlation': selected_corr
    })

    return df_best, corr_scores


def lasso_minimize_features(feature_table):
    """
    Uses Lasso regression to minimize the number of features.
    It selects features based on their coefficients and saves the
    selected features to a CSV file.

    TODO: This isn't currently working it's removing data samples for some reason.
    """

    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    feature_names = [name.replace('_', ' ').title() for name in feature_names]
    labels = feature_table['Label'].values

    scaler = StandardScaler()
    feature_array = scaler.fit_transform(feature_array)

    X_train, X_test, y_train, y_test = train_test_split(
        feature_array, labels, test_size=0.20, random_state=42, stratify=labels)

    # parameters to be tested on GridSearchCV
    params = {"alpha": np.logspace(-6, 1, 100)}
    lasso_cv = LassoCV(alphas=np.logspace(-6, 1, 100), cv=5, max_iter=100000)
    lasso_cv.fit(X_train, y_train)

    # Get coefficients and feature names
    coef = lasso_cv.coef_
    names = feature_names
    selected_features = [name for coef_value, name in zip(coef, names) if coef_value != 0]

    # calling the model with the best parameter
    lasso1 = Lasso(alpha=lasso_cv.alpha_, max_iter=100000)
    lasso1.fit(X_train, y_train)

    # Using np.abs() to make coefficients positive.  
    lasso1_coef = np.abs(lasso1.coef_)

    # Subsetting the features which has more than 0.001 importance.
    feature_subset=np.array(names)[lasso1_coef>0.001]

    # Adding the target to the list of feaatures. 
    feature_subset=np.append(feature_subset, "Label")

    # Creating a new dataframe with the selected features.
    df_new = pd.DataFrame(X_test, columns=names)
    df_new['Label'] = y_test
    df_new = df_new[feature_subset]

    # Results
    results = pd.DataFrame({
        'Feature': feature_subset[:-1], 
        'Lasso Coefficients': lasso1_coef[lasso1_coef > 0.001]
    })

    return df_new, results

def save_feature_table(feature_table, destination, feature_file_name='features_selected.csv'):
    """
    Saves the feature table to a CSV file.
    """

    if not os.path.exists(destination):
        os.makedirs(destination)
    if os.path.exists(os.path.join(destination, feature_file_name)):
        print(f"File {feature_file_name} already exists in {destination}. Overwriting.")
    feature_table.to_csv(os.path.join(destination, feature_file_name), index=False)
    print(f"Feature table saved to {os.path.join(destination, feature_file_name)}")

def load_feature_table(dir, feature_file_name='features.csv'):
    """
    Loads the feature table from a CSV file.
    """

    feature_table = pd.read_csv(f"{dir}/{feature_file_name}")

    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    feature_names = [name.replace('_', ' ').title() for name in feature_names]
    labels = feature_table['Label'].values

    return feature_table, feature_array, feature_names, labels