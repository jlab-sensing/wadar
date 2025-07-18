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

    print("Test harness for all features related to the phase of the signal.")

    dataset_dir = "../../data/combined-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    hephaestus_features = feature_tools.FeatureTools(X)

    feature_array = np.zeros((X.shape[0], 0))
    feature_names = []

    # ==============
    # Phase at the two peaks
    # ==============

    peak_phase = hephaestus_features.peak_phase()

    tent_names = ["Peak Phase 1", "Peak Phase 2"]
    feature_array = np.hstack((feature_array, peak_phase))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Phase at Peaks", tent_names, peak_phase)

    # ==============
    # Phase Variance
    # ==============

    phase_variance = hephaestus_features.peak_phase_variance()

    tent_names = ["Phase Variance 1", "Phase Variance 2"]
    feature_array = np.hstack((feature_array, phase_variance))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Phase Variance", tent_names, phase_variance)

    # ==============
    # Circularity Coefficient
    # ==============

    circularity_coefficient = hephaestus_features.peak_circularity_coefficient()

    tent_names = ["Circularity Coefficient 1", "Circularity Coefficient 2"]
    feature_array = np.hstack((feature_array, circularity_coefficient))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Circularity Coefficient", tent_names, circularity_coefficient)

    # ==============
    # Phase Jitter
    # ==============

    peak_phase_jitter = hephaestus_features.peak_phase_jitter()

    tent_names = ["Phase Jitter 1", "Phase Jitter 2"]
    feature_array = np.hstack((feature_array, peak_phase_jitter))
    feature_names.append(tent_names[0])
    feature_names.append(tent_names[1])

    plot_features(VIZ, y, "Phase Jitter", tent_names, peak_phase_jitter)

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