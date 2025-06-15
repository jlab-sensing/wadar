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

    temp = feature_tools.FeatureTools(X, soil_index=200)
    peak_amplitude = temp.peak_amplitude()

    signal_variance = temp.signal_variance()
    signal_entropy = temp.signal_entropy()
    
    plt.figure()
    plt.plot(y, peak_amplitude, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Peak Amplitude')
    plt.title('Peak Amplitude vs Bulk Density')
    plt.grid()

    plt.figure()
    plt.plot(y, signal_variance, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Signal Variance')
    plt.title('Signal Variance vs Bulk Density')
    plt.grid()

    plt.figure()
    plt.plot(y, signal_entropy, 'o')
    plt.xlabel('Bulk Density')
    plt.ylabel('Signal Features')
    plt.title('Signal Entropy vs Bulk Density')
    plt.grid()

    plt.show()

    # viz_tools.plot_median_unique(X, y)