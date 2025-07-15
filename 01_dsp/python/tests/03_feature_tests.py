import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools
import seaborn as sns

def eval_corr(y, feature):
    """
    Evaluates the correlation between the feature and the labels.
    """
    corr = np.corrcoef(y, feature)[0, 1]
    return corr

if __name__ == "__main__":

    VIZ = True

    dataset_dir = "../data/dry-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    feature_tools = feature_tools.FeatureTools(X)

    # ==========
    # Peak Amplitude
    # ==========

    peak_amplitude = feature_tools.peak_amplitude()

    print("Peak Amplitude Correlation:")
    print("Correlation with Peak Amplitude 1:", eval_corr(y, peak_amplitude[:, 0]))
    print("Correlation with Peak Amplitude 2:", eval_corr(y, peak_amplitude[:, 1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, peak_amplitude[:, 0], 'o', label='Peak Amplitude 1')
        plt.plot(y, peak_amplitude[:, 1], 'o', label='Peak Amplitude 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Amplitude')
        plt.title('Peak Amplitude vs Soil Compaction')
        plt.legend()

    # ==========
    # Find hte range bins with peak variances correlated most with the labels.
    # This is commented out because it takes forever to run.
    # ==========

    # corr_coefs = []
    # for i in range(X.shape[1]):
    #     signal_variance = feature_tools.signal_variance(i)
    #     corr_coef = eval_corr(y, signal_variance)
    #     corr_coefs.append((corr_coef, i))

    # num_of_corrs = 10
    # # Sort by absolute correlation (already correct)
    # top_corrs = sorted(corr_coefs, key=lambda x: abs(x[0]), reverse=True)[:num_of_corrs]

    # print(f"Peak Variance - Top {num_of_corrs} Correlations:")
    # for rank, (coef, idx) in enumerate(top_corrs, 1):
    #     print(f"{rank}. Correlation Coefficient: {coef:.4f} @ {idx}")
    # print()

    # if VIZ:
    #     plt.figure(figsize=(10, 6))
    #     for coef, idx in top_corrs:
    #         sns.scatterplot(x=y, y=feature_tools.signal_variance(idx), label=f'Idx {idx} (r={coef:.2f})')
    #     plt.xlabel('Soil Compaction Level')
    #     plt.ylabel('Peak Variance')
    #     plt.title(f'Top {num_of_corrs} Correlated Peak Variance Features')
    #     plt.legend()

    # ==========
    # Peak Variance
    # ==========

    print("Peak Variance Correlation:")
    peak_variances = feature_tools.peak_variance()
    peak_1_variance = peak_variances[0]
    peak_2_variance = peak_variances[1]
    print("Correlation with Peak Variance 1:", eval_corr(y, peak_1_variance))
    print("Correlation with Peak Variance 2:", eval_corr(y, peak_2_variance))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, peak_1_variance, 'o', label='Peak Variance 1')
        plt.plot(y, peak_2_variance, 'o', label='Peak Variance 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Variance')
        plt.title('Peak Variance vs Soil Compaction')
        plt.legend()

    # ==========
    # Peak amplitude to peak variance ratios
    # ==========

    peak_amplitude_variance_ratio = feature_tools.peak_amplitude2variance_ratio()
    print("Peak Amplitude to Peak Variance Ratio Correlation:")
    print("Correlation with Peak Amplitude to Peak Variance Ratio 1:", eval_corr(y, peak_amplitude_variance_ratio[0]))
    print("Correlation with Peak Amplitude to Peak Variance Ratio 2:", eval_corr(y, peak_amplitude_variance_ratio[1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, peak_amplitude_variance_ratio[0], 'o', label='Peak Amplitude to Peak Variance Ratio 1')
        plt.plot(y, peak_amplitude_variance_ratio[1], 'o', label='Peak Amplitude to Peak Variance Ratio 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Amplitude to Peak Variance Ratio')
        plt.title('Peak Amplitude to Peak Variance Ratio vs Soil Compaction')
        plt.legend()

    # ==========
    # Find the range bin with the peak entropy correlated most with the labels.
    # ==========

    # corr_coefs = []
    # for i in range(X.shape[1]):
    #     signal_entropy = feature_tools.signal_entropy(i)
    #     corr_coef = eval_corr(y, signal_entropy)
    #     corr_coefs.append((corr_coef, i))

    # num_of_corrs = 10
    # top_corrs = sorted(corr_coefs, key=lambda x: abs(x[0]), reverse=True)[:num_of_corrs]
    # print(f"Peak Entropy - Top {num_of_corrs} Correlations:")
    # for rank, (coef, idx) in enumerate(top_corrs, 1):
    #     print(f"{rank}. Correlation Coefficient: {coef:.4f} @ {idx}")
    # print()

    # if VIZ:
    #     plt.figure(figsize=(10, 6))
    #     for coef, idx in top_corrs:
    #         sns.scatterplot(x=y, y=feature_tools.signal_entropy(idx), label=f'Idx {idx} (r={coef:.2f})')
    #     plt.xlabel('Soil Compaction Level')
    #     plt.ylabel('Peak Entropy')
    #     plt.title(f'Top {num_of_corrs} Correlated Peak Entropy Features')
    #     plt.legend()
    
    # ==========
    # Peak Entropy
    # ==========

    print("Peak Entropy Correlation:")
    peak_entropy = feature_tools.peak_entropy()
    print("Correlation with Peak Entropy 1:", eval_corr(y, peak_entropy[0]))
    print("Correlation with Peak Entropy 2:", eval_corr(y, peak_entropy[1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, peak_entropy[0], 'o', label='Peak Entropy 1')
        plt.plot(y, peak_entropy[1], 'o', label='Peak Entropy 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Entropy')
        plt.title('Peak Entropy vs Soil Compaction')
        plt.legend()

    # ==========
    # Peak Amplitude to Peak Entropy Ratios
    # ==========

    peak_amplitude_entropy_ratio = feature_tools.peak_amplitude2entropy_ratio()
    print(peak_amplitude_entropy_ratio.shape)
    print("Peak Amplitude to Peak Entropy Ratio Correlation:")
    print("Correlation with Peak Amplitude to Peak Entropy Ratio 1:", eval_corr(y, peak_amplitude_entropy_ratio[0]))
    print("Correlation with Peak Amplitude to Peak Entropy Ratio 2:", eval_corr(y, peak_amplitude_entropy_ratio[1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, peak_amplitude_entropy_ratio[0], 'o', label='Peak Amplitude to Peak Entropy Ratio 1')
        plt.plot(y, peak_amplitude_entropy_ratio[1], 'o', label='Peak Amplitude to Peak Entropy Ratio 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Amplitude to Peak Entropy Ratio')
        plt.title('Peak Amplitude to Peak Entropy Ratio vs Soil Compaction')
        plt.legend()

    if VIZ:  # This function calls plt.show() internally so don't call it anywhere else
        viz_tools.plot_median_unique(X, y)