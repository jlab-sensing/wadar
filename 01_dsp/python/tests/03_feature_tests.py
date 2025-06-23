import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools


if __name__ == "__main__":

    dataset_dir = "../data/compact-4-dry"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

            
    # signal_horizontal = X[:, 200, :]  
    # print("Signal shape:", signal_horizontal.shape)
    
    # plt.figure(figsize=(10, 6))
    # plt.plot(signal_horizontal[0, :], label=y[0])  # Plot the first signal
    # plt.plot(signal_horizontal[20, :], label=y[20])  # Plot the 21st signal
    # plt.plot(signal_horizontal[40, :], label=y[40])  # Plot the 41st signal
    # plt.xlabel('Range Bins')
    # plt.ylabel('Amplitude')
    # plt.title('Horizontal Signal Samples')
    # plt.legend()
    # plt.grid()
    # plt.show()

    features = feature_tools.FeatureTools(X, soil_index=200)

    peak_amplitude = features.peak_amplitude()
    plt.figure()
    plt.plot(y, peak_amplitude, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Peak Amplitude')
    plt.title('Peak Amplitude vs Bulk Density')
    plt.grid()

    signal_variance = features.signal_variance()
    plt.figure()
    plt.plot(y, signal_variance, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Signal Variance')
    plt.title('Signal Variance vs Bulk Density')
    plt.grid()

    signal_entropy = features.signal_entropy()
    plt.figure()
    plt.plot(y, signal_entropy, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Signal Entropy')
    plt.title('Signal Entropy vs Bulk Density')
    plt.grid()

    signal_energy = features.signal_energy()
    plt.figure()
    plt.plot(y, signal_energy, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Signal Energy')
    plt.title('Signal Energy vs Bulk Density')
    plt.grid()

    # peak_delay = features.peak_delay()            # Not currently accurate.
    # plt.figure()
    # plt.plot(y, peak_delay, 'o')
    # plt.xlabel('Bulk Density')
    # plt.ylabel('Peak Delay')
    # plt.title('Peak Delay vs Bulk Density')
    # plt.grid()

    peak_width_fwhm = features.peak_width_fwhm()
    plt.figure()
    plt.plot(y, peak_width_fwhm, 'o')
    plt.xlabel('Bulk Density')  
    plt.ylabel('Peak Width FWHM')
    plt.title('Peak Width FWHM vs Bulk Density')
    plt.grid()

    signal_skewness, signal_kurtosis = features.signal_skewness_kurtosis()
    plt.figure()
    plt.plot(y, signal_skewness, 'o', label='Skewness')
    plt.plot(y, signal_kurtosis, 'o', label='Kurtosis')
    plt.xlabel('Bulk Density')
    plt.ylabel('Signal Skewness/Kurtosis')
    plt.title('Signal Skewness and Kurtosis vs Bulk Density')
    plt.legend()
    plt.grid()

    # centroid, bandwidth = features.spectral_centroid_bandwidth()          # Black box. Not going to use it until I understand it.
    # plt.figure()
    # plt.plot(y, centroid, 'o', label='Spectral Centroid')
    # plt.plot(y, bandwidth, 'o', label='Spectral Bandwidth')
    # plt.xlabel('Bulk Density')
    # plt.ylabel('Spectral Features')
    # plt.title('Spectral Centroid and Bandwidth vs Bulk Density')
    # plt.legend()
    # plt.grid()

    entropy_energy_ratio = features.entropy_to_energy_ratio()
    plt.figure()   
    plt.plot(y, entropy_energy_ratio, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Entropy to Energy Ratio')
    plt.title('Entropy to Energy Ratio vs Bulk Density')
    plt.grid()

    peak_entropy_ratio = features.peak_to_entropy_ratio()
    plt.figure()    
    plt.plot(y, peak_entropy_ratio, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Peak to Entropy Ratio')
    plt.title('Peak to Entropy Ratio vs Bulk Density')
    plt.grid()

    plt.show()

    # viz_tools.plot_median_unique(X, y)