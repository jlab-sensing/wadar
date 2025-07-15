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
from sklearn.linear_model import Lasso, LassoCV
from sklearn.model_selection import train_test_split
import pandas as pd
from sklearn.model_selection import GridSearchCV, KFold
from sklearn.preprocessing import StandardScaler

def eval_corr(y, feature):
    """
    Evaluates the correlation between the feature and the labels.
    """
    corr = np.corrcoef(y, feature)[0, 1]
    return corr

def eval_top_correlation(y, X, feature_func, num_of_corrs=10):

    corr_coefs = []
    for i in range(X.shape[1]):
        feature = feature_func(i)
        corr_coef = eval_corr(y, feature)
        corr_coefs.append((corr_coef, i))

    top_corrs = sorted(corr_coefs, key=lambda x: abs(x[0]), reverse=True)[:num_of_corrs]

    for rank, (coef, idx) in enumerate(top_corrs, 1):
        print(f"{rank}. Correlation Coefficient: {coef:.4f} @ {idx}")
    print()

if __name__ == "__main__":

    VIZ = False

    dataset_dir = "../data/dry-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    hephaestus_features = feature_tools.FeatureTools(X)

    # ==========
    # Peak Amplitude
    # ==========

    peak_amplitude = hephaestus_features.peak_amplitude()

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

    print("Peak Variance Correlation:")
    top_corrs = eval_top_correlation(y, X, hephaestus_features.signal_variance, num_of_corrs=10)

    if VIZ:
        plt.figure(figsize=(10, 6))
        for coef, idx in top_corrs:
            sns.scatterplot(x=y, y=hephaestus_features.signal_variance(idx), label=f'Idx {idx} (r={coef:.2f})')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Variance')
        plt.title(f'Top Correlated Peak Variance Features')
        plt.legend()

    # ==========
    # Peak Variance
    # ==========

    print("Peak Variance Correlation:")
    peak_variances = hephaestus_features.peak_variance()
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

    peak_amplitude_variance_ratio = hephaestus_features.peak_amplitude2variance_ratio()
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

    corr_coefs = []
    for i in range(X.shape[1]):
        signal_entropy = hephaestus_features.signal_entropy(i)
        corr_coef = eval_corr(y, signal_entropy)
        corr_coefs.append((corr_coef, i))

    num_of_corrs = 10
    top_corrs = sorted(corr_coefs, key=lambda x: abs(x[0]), reverse=True)[:num_of_corrs]
    print(f"Peak Entropy - Top {num_of_corrs} Correlations:")
    for rank, (coef, idx) in enumerate(top_corrs, 1):
        print(f"{rank}. Correlation Coefficient: {coef:.4f} @ {idx}")
    print()

    if VIZ:
        plt.figure(figsize=(10, 6))
        for coef, idx in top_corrs:
            sns.scatterplot(x=y, y=hephaestus_features.signal_entropy(idx), label=f'Idx {idx} (r={coef:.2f})')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Entropy')
        plt.title(f'Top Correlated Peak Entropy Features')
        plt.legend()
    
    # ==========
    # Peak Entropy
    # ==========

    print("Peak Entropy Correlation:")
    peak_entropy = hephaestus_features.peak_entropy()
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

    peak_amplitude_entropy_ratio = hephaestus_features.peak_amplitude2entropy_ratio()
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

    # ==========
    # Peak delays
    # ==========

    peak_delays = hephaestus_features.peak_delay()
    print("Peak Delays Correlation:")
    print("Correlation with Peak Delay 1:", eval_corr(y, peak_delays[0]))
    print("Correlation with Peak Delay 2:", eval_corr(y, peak_delays[1]))
    print("Correlation with Peak Delay 3:", eval_corr(y, peak_delays[2]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, peak_delays[0], 'o', label='Peak Delay 1')
        plt.plot(y, peak_delays[1], 'o', label='Peak Delay 2')
        plt.plot(y, peak_delays[2], 'o', label='Peak Delay 3')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Delay')
        plt.title('Peak Delays vs Soil Compaction')
        plt.legend() 

    # ==========
    # Peak widths
    # ==========

    peak_widths = hephaestus_features.peak_width()
    print("Peak Widths Correlation:")
    print("Correlation with Peak Width 1:", eval_corr(y, peak_widths[0]))
    print("Correlation with Peak Width 2:", eval_corr(y, peak_widths[1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, peak_widths[0], 'o', label='Peak Width 1')
        plt.plot(y, peak_widths[1], 'o', label='Peak Width 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Width')
        plt.title('Peak Widths vs Soil Compaction')
        plt.legend()
    
    # ==========
    # Peak shapes
    # ==========

    skewness, kurtosis = hephaestus_features.peak_shape_stats()

    print("Peak Shapes Correlation:")
    print("Correlation with Peak Shape Skewness 1:", eval_corr(y, skewness[0]))
    print("Correlation with Peak Shape Skewness 2:", eval_corr(y, skewness[1]))
    print("Correlation with Peak Shape Kurtosis 1:", eval_corr(y, kurtosis[0]))
    print("Correlation with Peak Shape Kurtosis 2:", eval_corr(y, kurtosis[1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, skewness[0], 'o', label='Peak Shape Skewness 1')
        plt.plot(y, skewness[1], 'o', label='Peak Shape Skewness 2')
        plt.plot(y, kurtosis[0], 'o', label='Peak Shape Kurtosis 1')
        plt.plot(y, kurtosis[1], 'o', label='Peak Shape Kurtosis 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Shape')
        plt.title('Peak Shapes vs Soil Compaction')
        plt.legend()

    # ==========
    # Peak energy
    # ==========

    peak_energy = hephaestus_features.peak_signal_energy()
    print("Peak Energy Correlation:")
    print("Correlation with Peak Energy 1:", eval_corr(y, peak_energy[0]))
    print("Correlation with Peak Energy 2:", eval_corr(y, peak_energy[1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, peak_energy[0], 'o', label='Peak Energy 1')
        plt.plot(y, peak_energy[1], 'o', label='Peak Energy 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Peak Energy')
        plt.title('Peak Energy vs Soil Compaction')
        plt.legend()

    # ==========
    # Decay rate
    # ==========

    decay_rate = hephaestus_features.decay_rate()
    print("Decay Rate Correlation:")
    print("Correlation with Decay Rate 1:", eval_corr(y, decay_rate[0]))
    print("Correlation with Decay Rate 2:", eval_corr(y, decay_rate[1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, decay_rate[0], 'o', label='Decay Rate 1')
        plt.plot(y, decay_rate[1], 'o', label='Decay Rate 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Decay Rate')
        plt.title('Decay Rate vs Soil Compaction')
        plt.legend()

    # ==========
    # Ascend rate
    # ==========

    ascend_rate = hephaestus_features.ascend_rate()
    print("Ascend Rate Correlation:")
    print("Correlation with Ascend Rate 1:", eval_corr(y, ascend_rate[0]))
    print("Correlation with Ascend Rate 2:", eval_corr(y, ascend_rate[1]))
    print()

    if VIZ:
        plt.figure()
        plt.plot(y, ascend_rate[0], 'o', label='Ascend Rate 1')
        plt.plot(y, ascend_rate[1], 'o', label='Ascend Rate 2')
        plt.xlabel('Soil Compaction Level')
        plt.ylabel('Ascend Rate')
        plt.title('Ascend Rate vs Soil Compaction')
        plt.legend()

    # ==========
    # Peak phrase variance
    # ==========

    peak_phase_variance = hephaestus_features.peak_phase_variance()
    print("Peak Phase Variance Correlation:")
    print("Correlation with Peak Phase Variance 1:", eval_corr(y, peak_phase_variance[0]))
    print("Correlation with Peak Phase Variance 2:", eval_corr(y, peak_phase_variance[1]))

    top_corrs = eval_top_correlation(y, X, hephaestus_features.phase_variance, num_of_corrs=10)
    print()

    # ==========
    # Circularity coefficient
    # ==========

    circularity_coefficient = hephaestus_features.peak_circularity_coefficient()
    print("Circularity Coefficient Correlation:")
    print("Correlation with Circularity Coefficient 1:", eval_corr(y, circularity_coefficient[0]))
    print("Correlation with Circularity Coefficient 2:", eval_corr(y, circularity_coefficient[1]))

    top_corrs = eval_top_correlation(y, X, hephaestus_features.circularity_coefficient, num_of_corrs=10)

    # ==========
    # Phase jitter
    # ==========

    phase_jitter = hephaestus_features.peak_phase_jitter()
    print("Phase Jitter Correlation:")
    print("Correlation with Phase Jitter 1:", eval_corr(y, phase_jitter[0]))
    print("Correlation with Phase Jitter 2:", eval_corr(y, phase_jitter[1]))

    top_corrs = eval_top_correlation(y, X, hephaestus_features.phase_jitter, num_of_corrs=10)

        
    # ==========
    # Plot median frame
    # ==========

    # This function calls plt.show() internally so don't call it anywhere else
    viz_tools.plot_median_unique(X, y)

    # ==========
    # Save all features and commit feature selection using lasso regression
    # ==========

    # df = feature_tools.get_feature_dataframe(X, y, destination=dataset_dir)

    # # Sourced from 
    # # https://medium.com/@agrawalsam1997/feature-selection-using-lasso-regression-10f49c973f08
    # X = df.drop(columns=['label']).values
    # y = df['label'].values

    # scaler = StandardScaler()
    # X = scaler.fit_transform(X)

    # X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.20, random_state=42, stratify=y)

    # print("Shape of Train Features: {}".format(X_train.shape))
    # print("Shape of Test Features: {}".format(X_test.shape))
    # print("Shape of Train Target: {}".format(y_train.shape))
    # print("Shape of Test Target: {}".format(y_test.shape))



    # # parameters to be tested on GridSearchCV
    # params = {"alpha": np.logspace(-6, 1, 100)}
    # lasso_cv = LassoCV(alphas=np.logspace(-6, 1, 100), cv=5, max_iter=100000)
    # lasso_cv.fit(X_train, y_train)
    # print("Best Alpha:", lasso_cv.alpha_)

    # # Get coefficients and feature names
    # coef = lasso_cv.coef_
    # names = df.drop(columns=['label']).columns
    # selected_features = [name for coef_value, name in zip(coef, names) if coef_value != 0]
    # print("Selected Features:", selected_features)

    # # calling the model with the best parameter
    # lasso1 = Lasso(alpha=lasso_cv.alpha_, max_iter=100000)
    # lasso1.fit(X_train, y_train)

    # # Using np.abs() to make coefficients positive.  
    # lasso1_coef = np.abs(lasso1.coef_)

    # # plotting the Column Names and Importance of Columns. 
    # plt.bar(names, lasso1_coef)
    # plt.xticks(rotation=90)
    # plt.grid()
    # plt.title("Feature Selection Based on Lasso")
    # plt.xlabel("Features")
    # plt.ylabel("Importance")
    # plt.ylim(0, 0.15)
    # plt.show()

    # # Subsetting the features which has more than 0.001 importance.
    # feature_subset=np.array(names)[lasso1_coef>0.001]
    # print("Selected Feature Columns: {}".format(feature_subset))

    # # Adding the target to the list of feaatures. 
    # feature_subset=np.append(feature_subset, "Outcome")
    # print("Selected Columns: {}".format(feature_subset))