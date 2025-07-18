import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools
import seaborn as sns
import pandas as pd
from sklearn.feature_selection import mutual_info_regression

def eval_corr(y, feature):
    """
    Evaluates the correlation between the feature and the labels.
    """
    corr = np.corrcoef(y, feature)[0, 1]
    return corr

def plot_features(VIZ, y, feature_title, feature_names, feature_array):
    if not VIZ:
        return
    plt.figure(figsize=(8, 6))
    for i in range(feature_array.shape[1]):
        sns.scatterplot(x=y, y=feature_array[:, i], label=feature_names[i], s=60, edgecolor='k')
    plt.xlabel('Soil Compaction Level', fontsize=14)
    plt.ylabel(f'{feature_title}', fontsize=14)
    # plt.title(f'{feature_title} vs Soil Compaction', fontsize=16, fontweight='bold')
    plt.legend(fontsize=12)
    plt.tight_layout()

if __name__ == "__main__":

    VIZ = False

    print("Test harness for all features related to the magnitude of the signal.")

    dataset_dir = "../../data/combined-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    hephaestus_features = feature_tools.FeatureTools(X)

    feature_array = np.zeros((X.shape[0], 0))
    feature_names = []

    # viz_tools.plot_IQ_signals(X, y)

    # ==============
    # Amplitude at the two peaks
    # ==============

    peak_amplitude = hephaestus_features.peak_amplitude()

    tent_names = ["Peak Amplitude 1", "Peak Amplitude 2"]
    feature_array = np.hstack((feature_array, peak_amplitude))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Peak Amplitude", tent_names, peak_amplitude)

    # ==============
    # Variance
    # ==============

    peak_variance = hephaestus_features.peak_variance()

    tent_names = ["Peak Variance 1", "Peak Variance 2"]
    feature_array = np.hstack((feature_array, peak_variance))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])
    
    plot_features(VIZ, y, "Peak Variance", tent_names, peak_variance)

    amplitude2variance_ratio = peak_amplitude / (peak_variance + 1e-10)
    feature_array = np.hstack((feature_array, amplitude2variance_ratio))
    feature_names.append("Amplitude to Variance Ratio 1")
    feature_names.append("Amplitude to Variance Ratio 2")
    plot_features(VIZ, y, "Amplitude to Variance Ratio", ["Amplitude to Variance Ratio 1", "Amplitude to Variance Ratio 2"], amplitude2variance_ratio)

    # ==============
    # Signal entropy 
    # ==============

    peak_entropy = hephaestus_features.peak_entropy()

    tent_names = ["Peak Entropy 1", "Peak Entropy 2"]
    feature_array = np.hstack((feature_array, peak_entropy))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Peak Entropy", tent_names, peak_entropy)

    amplitude2entropy_ratio = peak_amplitude / (peak_entropy + 1e-10)

    feature_array = np.hstack((feature_array, amplitude2entropy_ratio))
    feature_names.append("Amplitude to Entropy Ratio 1")
    feature_names.append("Amplitude to Entropy Ratio 2")
    plot_features(VIZ, y, "Amplitude to Entropy Ratio", ["Amplitude to Entropy Ratio 1", "Amplitude to Entropy Ratio 2"], amplitude2entropy_ratio)

    # ==============
    # Peak delay
    # ==============

    peak_delay = hephaestus_features.peak_delay()

    tent_names = ["Peak Delay 1", "Peak Delay 2", "Peak Delay 3"]
    feature_array = np.hstack((feature_array, peak_delay))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])
    feature_names.append(tent_names[2])

    plot_features(VIZ, y, "Peak Delay", tent_names, peak_delay)

    # ==============
    # Peak width
    # ==============

    peak_width = hephaestus_features.peak_width()

    tent_names = ["Peak Width 1", "Peak Width 2"]
    feature_array = np.hstack((feature_array, peak_width))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Peak Width", tent_names, peak_width)

    # ==============
    # Peak shape statistics
    # ==============

    skewness, kurtosis = hephaestus_features.peak_shape_stats()

    tent_names = ["Skewness 1", "Skewness 2", "Kurtosis 1", "Kurtosis 2"]
    feature_array = np.hstack((feature_array, skewness, kurtosis))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])
    feature_names.append(tent_names[2])
    feature_names.append(tent_names[3])

    plot_features(VIZ, y, "Peak Shape Statistics", tent_names, np.hstack((skewness, kurtosis)))

    # ==============
    # Signal energy
    # ==============

    signal_energy = hephaestus_features.peak_signal_energy()

    tent_names = ["Signal Energy 1", "Signal Energy 2"]
    feature_array = np.hstack((feature_array, signal_energy))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Signal Energy", tent_names, signal_energy)

    # ==============
    # Decay rate
    # ==============

    decay_rate = hephaestus_features.decay_rate()

    tent_names = ["Decay Rate 1", "Decay Rate 2"]
    feature_array = np.hstack((feature_array, decay_rate))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Decay Rate", tent_names, decay_rate)

    # ==============
    # Ascend rate
    # ==============

    ascend_rate = hephaestus_features.ascend_rate()

    tent_names = ["Ascend Rate 1", "Ascend Rate 2"]
    feature_array = np.hstack((feature_array, ascend_rate))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Ascend Rate", tent_names, ascend_rate)

    # ==============
    # Assembling the feature table
    # ==============

    feature_df = pd.DataFrame({
        'Feature': feature_names,
        'Values': np.hstack(feature_array)
    })

    # ==============
    # Mutual Information for feature selection
    # ==============

    mi = mutual_info_regression(feature_array, y)
    corr = np.array([eval_corr(y, feature_array[:, i]) for i in range(feature_array.shape[1])]) # relevant for linear relationships

    feature_df = pd.DataFrame({
        'Feature': feature_names,
        'Mutual Information': mi,
        'Correlation': corr})
    feature_df = feature_df.sort_values(by='Mutual Information', ascending=False)
    print("Mutual Information of Features:")
    print(feature_df)



    if VIZ:
        plt.figure(figsize=(10, 6))
        sns.barplot(x='Feature', y='Mutual Information', data=feature_df)
        plt.xticks(rotation=45)
        plt.xlabel('Feature Names')
        plt.ylabel('Mutual Information')
        plt.title('Mutual Information of Features')
        plt.tight_layout()
        plt.ylim([0, 1])

        plt.show()