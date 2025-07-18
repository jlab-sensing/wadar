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
            
    def _get_phase_variance(self, scan_idx=0, spec_idx=0):
        """
        Computes the variance of the phase of the signal at scan_idx.
        This is because the stochastic nature of the signal reflecting
        on more porous soil may affect the phase of the signal.
        """

        signal = self.X[scan_idx, :, :][spec_idx]
        phase = np.angle(signal)
        return np.var(phase)

    def phase_variance(self, spec_idx=0):
        """
        Computes the variance of the phase of the signal for all scans.
        """

        variance = np.zeros(self.X.shape[0])
        for i in range(self.X.shape[0]):
            variance[i] = self._get_phase_variance(i, spec_idx)
        return variance

    def peak_phase_variance(self):
        """
        Computes the phase variance of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        return [
            self.phase_variance(peak_idxs[0]),
            self.phase_variance(peak_idxs[1])
        ]

    def _get_circularity_coefficient(self, scan_idx=0, spec_idx=0):
        """
        Computes the circularity coefficient of the signal at scan_idx.
        The circularity coefficient describes the statistical
        scattering behavior of the signal. 
        """

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
        """
        Computes the circularity coefficient of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        return [
            self.circularity_coefficient(peak_idxs[0]),
            self.circularity_coefficient(peak_idxs[1])
        ]

    def _get_phase_jitter(self, scan_idx=0, spec_idx=0):
        """
        Computes the phase jitter of the signal at scan_idx.
        Phase jitter is the variance of the phase difference
        between consecutive samples in the signal.
        """

        signal = self.X[scan_idx, :, :][spec_idx]
        phase = np.unwrap(np.angle(signal))
        return np.var(np.diff(phase))

    def phase_jitter(self, spec_idx=0):
        """
        Computes the phase jitter of the signal for all scans.
        """

        jitter = np.zeros(self.X.shape[0])
        for i in range(self.X.shape[0]):
            jitter[i] = self._get_phase_jitter(i, spec_idx)
        return jitter

    def peak_phase_jitter(self):
        """
        Computes the phase jitter of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        return [
            self.phase_jitter(peak_idxs[0]),
            self.phase_jitter(peak_idxs[1])
        ]
    
    def _get_phase(self, spec_idx=0):
        """
        Computes the phase of the signal at the specified index.
        The phase is the angle of the complex signal.
        """

        phase = np.angle(self.X)
        phase = np.median(phase[:, spec_idx], axis=1)
        return phase

    def peak_phase(self):
        """
        Computes the phase of the two peaks.
        """

        peak_idxs = self._get_peak_idx()
        return [
            self._get_phase(peak_idxs[0]),
            self._get_phase(peak_idxs[1])
        ]
    
def get_feature_dataframe(X, labels, destination=None):
    """
    Generates a DataFrame with various features extracted from the input data.
    """

    ft = FeatureTools(X)

    peak_amps = ft.peak_amplitude()                     # shape (N, 2)
    peak_vars = ft.peak_variance()                      # list of two arrays (N,)
    peak_entropy = ft.peak_entropy()                    # list of two arrays (N,)
    peak_amp2var = ft.peak_amplitude2variance_ratio()   # shape (2, N)
    peak_amp2ent = ft.peak_amplitude2entropy_ratio()    # shape (2, N)
    peak_delay = ft.peak_delay()                        # shape (3, N)
    peak_width = ft.peak_width()                        # shape (2, N)
    peak_skew, peak_kurt = ft.peak_shape_stats()        # both (2, N)
    peak_energy = ft.peak_signal_energy()               # list of two arrays (N,)
    decay_rate = ft.decay_rate()                        # shape (2, N)
    ascend_rate = ft.ascend_rate()                      # shape (2, N)
    phase_var = ft.peak_phase_variance()                # list of two arrays (N,)
    circ_coeff = ft.peak_circularity_coefficient()      # list of two arrays (N,)
    phase_jitter = ft.peak_phase_jitter()               # list of two arrays (N,)
    phase = ft.peak_phase()

    feature_dict = {
        'amp1': peak_amps[:, 0],
        'amp2': peak_amps[:, 1],
        'var1': peak_vars[0],
        'var2': peak_vars[1],
        'entropy1': peak_entropy[0],
        'entropy2': peak_entropy[1],
        'amp2var1': peak_amp2var[0, :],
        'amp2var2': peak_amp2var[1, :],
        'amp2ent1': peak_amp2ent[0, :],
        'amp2ent2': peak_amp2ent[1, :],
        'delay1': peak_delay[0, :],
        'delay2': peak_delay[1, :],
        'delay_diff': peak_delay[2, :],
        'width1': peak_width[0, :],
        'width2': peak_width[1, :],
        'skew1': peak_skew[0, :],
        'skew2': peak_skew[1, :],
        'kurt1': peak_kurt[0, :],
        'kurt2': peak_kurt[1, :],
        'energy1': peak_energy[0],
        'energy2': peak_energy[1],
        'decay1': decay_rate[0, :],
        'decay2': decay_rate[1, :],
        'ascend1': ascend_rate[0, :],
        'ascend2': ascend_rate[1, :],
        'phase_var1': phase_var[0],
        'phase_var2': phase_var[1],
        'circ_coeff1': circ_coeff[0],
        'circ_coeff2': circ_coeff[1],
        'phase_jitter1': phase_jitter[0],
        'phase_jitter2': phase_jitter[1],
        'phase1': phase[0],
        'phase2': phase[1],
        'label': labels
    }

    df = pd.DataFrame(feature_dict)

    if destination is not None:
        if not os.path.exists(destination):
            os.makedirs(destination)
        df.to_csv(os.path.join(destination, 'features.csv'), index=False)

    return df

def get_feature_dataframe_all(X, labels, destination=None):
    """
    Generates a DataFrame with various features extracted from the input data.
    This function is similar to get_feature_dataframe but does not save the DataFrame.
    """

    print("Initializing FeatureTools...")
    ft = FeatureTools(X)

    print("Extracting amplitudes...")
    amplitudes = np.zeros((X.shape[0], X.shape[1]))
    for i in range(X.shape[1]):
        amplitudes[:, i] = ft._get_amplitude(i)
    amp_dict = {f'amp{i+1}': amplitudes[:, i] for i in range(amplitudes.shape[1])}

    print("Extracting variance...")
    var = np.zeros((X.shape[0], X.shape[1]))
    for i in range(X.shape[1]):
        var[:, i] = ft.signal_variance(i)
    var_dict = {f'var{i+1}': var[:, i] for i in range(var.shape[1])}

    print("Extracting entropy...")
    entropy = np.zeros((X.shape[0], X.shape[1]))
    for i in range(X.shape[1]):
        entropy[:, i] = ft.signal_entropy(i)
    entropy_dict = {f'entropy{i+1}': entropy[:, i] for i in range(entropy.shape[1])}

    # print("Extracting amplitude to variance ratio...")
    # amp2var = np.zeros((X.shape[0], X.shape[1]))
    # for i in range(X.shape[1]):
    #     amp2var[:, i] = ft._get_amplitude2variance_ratio(i)
    # amp2var_dict = {f'amp2var{i+1}': amp2var[i, :] for i in range(amp2var.shape[0])}

    # print("Extracting amplitude to entropy ratio...")
    # amp2ent = np.zeros((X.shape[0], X.shape[1]))
    # for i in range(X.shape[1]):
    #     amp2ent[:, i] = ft._get_amplitude2entropy_ratio(i)
    # amp2ent_dict = {f'amp2ent{i+1}': amp2ent[i, :] for i in range(amp2ent.shape[0])}

    print("Extracting delay...")
    delay = ft.peak_delay()  # shape (3, N)
    delay_dict = {f'delay{i+1}': delay[i, :] for i in range(delay.shape[0])}

    print("Extracting peak width...")
    width = ft.peak_width()  # shape (2, N)
    width_dict = {f'width{i+1}': width[i, :] for i in range(width.shape[0])}

    print("Extracting skewness and kurtosis...")
    skewness, kurtosis_vals = ft.peak_shape_stats()  # both (2, N)
    skew_dict = {f'skew{i+1}': skewness[i, :] for i in range(skewness.shape[0])}
    kurt_dict = {f'kurt{i+1}': kurtosis_vals[i, :] for i in range(kurtosis_vals.shape[0])}

    print("Extracting energy...")
    energy = np.zeros((X.shape[0], X.shape[1]))
    for i in range(X.shape[0]):
        energy[:, i] = ft.signal_energy(i)
    energy_dict = {f'energy{i+1}': energy[:, i] for i in range(energy.shape[1])}

    print("Extracting decay rate...")
    decay = ft.decay_rate()  # shape (2, N)
    decay_dict = {f'decay{i+1}': decay[i, :] for i in range(decay.shape[0])}

    print("Extracting ascend rate...")
    ascend = ft.ascend_rate()  # shape (2, N)
    ascend_dict = {f'ascend{i+1}': ascend[i, :] for i in range(ascend.shape[0])}

    print("Extracting phase variance...")
    phase_var = np.zeros((X.shape[0], X.shape[1]))
    for i in range(X.shape[0]):
        phase_var[:, i] = ft.phase_variance(i)
    phase_var_dict = {f'phase_var{i+1}': phase_var[:, i] for i in range(phase_var.shape[1])}

    print("Extracting circularity coefficient...")
    circ_coeff = np.zeros((X.shape[0], X.shape[1]))
    for i in range(X.shape[0]):
        circ_coeff[:, i] = ft.circularity_coefficient(i)
    circ_coeff_dict = {f'circ_coeff{i+1}': circ_coeff[:, i] for i in range(circ_coeff.shape[1])}

    print("Extracting phase jitter...")
    phase_jitter = np.zeros((X.shape[0], X.shape[1]))
    for i in range(X.shape[0]):
        phase_jitter[:, i] = ft.phase_jitter(i)
    phase_jitter_dict = {f'phase_jitter{i+1}': phase_jitter[:, i] for i in range(phase_jitter.shape[1])}

    print("Merging all features into DataFrame...")
    feature_dict = {
        **amp_dict,
        **var_dict,
        **entropy_dict,
        # **amp2var_dict,
        # **amp2ent_dict,
        **delay_dict,
        **width_dict,
        **skew_dict,
        **kurt_dict,
        **energy_dict,
        **decay_dict,
        **ascend_dict,
        **phase_var_dict,
        **circ_coeff_dict,
        **phase_jitter_dict
    }

    feature_dict['label'] = labels

    df = pd.DataFrame(feature_dict)
    if destination is not None:
        print(f"Saving features to {os.path.join(destination, 'features.csv')} ...")
        if not os.path.exists(destination):
            os.makedirs(destination)
        df.to_csv(os.path.join(destination, 'features.csv'), index=False)
    
    print("Feature extraction complete.")
    return df

def correlation_minimize_features(dataset_dir, X, y, top_n=10):
    df = get_feature_dataframe_all(X, y, destination=dataset_dir)

    # Calculate correlation with the target variable
    corr = df.corr()['label'].abs().sort_values(ascending=False)

    # Select the top_n features most correlated with the label (excluding the label itself)
    top_features = corr.drop('label').head(top_n).index.tolist()
    selected_features = top_features + ['label']

    # Save the selected features to a new DataFrame
    df_new = df[selected_features]
    df_new.to_csv(os.path.join(dataset_dir, 'features_selected.csv'), index=False)

    # Return the top correlation scores as a dictionary
    top_corr_scores = corr[top_features].to_dict()
    return df_new, top_corr_scores

def lasso_minimize_features(dataset_dir, X, y):
    """
    Uses Lasso regression to minimize the number of features.
    It selects features based on their coefficients and saves the
    selected features to a CSV file.
    """
    df = get_feature_dataframe(X, y, destination=dataset_dir)

    # Sourced from 
    # https://medium.com/@agrawalsam1997/feature-selection-using-lasso-regression-10f49c973f08
    X = df.drop(columns=['label']).values
    y = df['label'].values

    scaler = StandardScaler()
    X = scaler.fit_transform(X)

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.20, random_state=42, stratify=y)



    # parameters to be tested on GridSearchCV
    params = {"alpha": np.logspace(-6, 1, 100)}
    lasso_cv = LassoCV(alphas=np.logspace(-6, 1, 100), cv=5, max_iter=100000)
    lasso_cv.fit(X_train, y_train)

    # Get coefficients and feature names
    coef = lasso_cv.coef_
    names = df.drop(columns=['label']).columns
    selected_features = [name for coef_value, name in zip(coef, names) if coef_value != 0]

    # calling the model with the best parameter
    lasso1 = Lasso(alpha=lasso_cv.alpha_, max_iter=100000)
    lasso1.fit(X_train, y_train)

    # Using np.abs() to make coefficients positive.  
    lasso1_coef = np.abs(lasso1.coef_)

    # Subsetting the features which has more than 0.001 importance.
    feature_subset=np.array(names)[lasso1_coef>0.001]

    # Adding the target to the list of feaatures. 
    feature_subset=np.append(feature_subset, "label")

    df_new = df[feature_subset]
    df_new.to_csv(os.path.join(dataset_dir, 'features_selected.csv'), index=False)

    # dictionary of feature names and their coefficients
    results = {name: coef_value for name, coef_value in zip(names, lasso1_coef) if coef_value != 0}

    return df_new, results