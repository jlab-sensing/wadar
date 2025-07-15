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

def get_feature_dataframe(X, labels, destination=None):
    ft = FeatureTools(X)

    peak_amps = ft.peak_amplitude()             # shape (N, 2)
    peak_vars = ft.peak_variance()              # list of two arrays (N,)
    peak_entropy = ft.peak_entropy()            # list of two arrays (N,)
    peak_amp2var = ft.peak_amplitude2variance_ratio()   # shape (2, N)
    peak_amp2ent = ft.peak_amplitude2entropy_ratio()    # shape (2, N)
    peak_delay = ft.peak_delay()                # shape (3, N)
    peak_width = ft.peak_width()                # shape (2, N)
    peak_skew, peak_kurt = ft.peak_shape_stats()   # both (2, N)
    peak_energy = ft.peak_signal_energy()       # list of two arrays (N,)
    decay_rate = ft.decay_rate()                # shape (2, N)
    ascend_rate = ft.ascend_rate()              # shape (2, N)
    phase_var = ft.peak_phase_variance()        # list of two arrays (N,)
    circ_coeff = ft.peak_circularity_coefficient()  # list of two arrays (N,)
    phase_jitter = ft.peak_phase_jitter()       # list of two arrays (N,)

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
        'label': labels
    }

    df = pd.DataFrame(feature_dict)

    if destination is not None:
        if not os.path.exists(destination):
            os.makedirs(destination)
        df.to_csv(os.path.join(destination, 'features.csv'), index=False)

    return df

def lasso_minimize_features(dataset_dir, X, y):
    df = get_feature_dataframe(X, y, destination=dataset_dir)

    # Sourced from 
    # https://medium.com/@agrawalsam1997/feature-selection-using-lasso-regression-10f49c973f08
    X = df.drop(columns=['label']).values
    y = df['label'].values

    scaler = StandardScaler()
    X = scaler.fit_transform(X)

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.20, random_state=42, stratify=y)

    print("Shape of Train Features: {}".format(X_train.shape))
    print("Shape of Test Features: {}".format(X_test.shape))
    print("Shape of Train Target: {}".format(y_train.shape))
    print("Shape of Test Target: {}".format(y_test.shape))



    # parameters to be tested on GridSearchCV
    params = {"alpha": np.logspace(-6, 1, 100)}
    lasso_cv = LassoCV(alphas=np.logspace(-6, 1, 100), cv=5, max_iter=100000)
    lasso_cv.fit(X_train, y_train)
    print("Best Alpha:", lasso_cv.alpha_)

    # Get coefficients and feature names
    coef = lasso_cv.coef_
    names = df.drop(columns=['label']).columns
    selected_features = [name for coef_value, name in zip(coef, names) if coef_value != 0]
    print("Selected Features:", selected_features)

    # calling the model with the best parameter
    lasso1 = Lasso(alpha=lasso_cv.alpha_, max_iter=100000)
    lasso1.fit(X_train, y_train)

    # Using np.abs() to make coefficients positive.  
    lasso1_coef = np.abs(lasso1.coef_)

    # plotting the Column Names and Importance of Columns. 
    plt.bar(names, lasso1_coef)
    plt.xticks(rotation=90)
    plt.grid()
    plt.title("Feature Selection Based on Lasso")
    plt.xlabel("Features")
    plt.ylabel("Importance")
    plt.ylim(0, 0.15)
    plt.show()

    # Subsetting the features which has more than 0.001 importance.
    feature_subset=np.array(names)[lasso1_coef>0.001]
    print("Selected Feature Columns: {}".format(feature_subset))

    # Adding the target to the list of feaatures. 
    feature_subset=np.append(feature_subset, "label")
    print("Selected Columns: {}".format(feature_subset))

    df_new = df[feature_subset]
    df_new.to_csv(os.path.join(dataset_dir, 'features_selected.csv'), index=False)