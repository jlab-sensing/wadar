import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
from dspml_pipeline.feature_extraction.learned.cnn import CNNLearnedFeatures
import numpy as np
from dspml_pipeline.plotting_tools.data_plotting import plot_feature_reduction
import matplotlib.pyplot as plt

from scipy import stats

def display_feature_importance(feature_array, feature_names, labels):
    correlations = []
    for i in range(feature_array.shape[1]):
        correlation, _ = stats.pointbiserialr(labels, feature_array[:, i])
        correlations.append((feature_names[i], correlation))

    correlations.sort(key=lambda x: abs(x[1]), reverse=True)

    for feature, corr in correlations:
        print(f"{feature}: {corr:.2f}")

if __name__ == "__main__":
    setup_logging(verbose=True)
    dataset_dirs = ["../../data/wet-0-soil-compaction-dataset",
                    "../../data/wet-1-soil-compaction-dataset",
                    "../../data/wet-2-soil-compaction-dataset"]
    target_dir = "../../data/training-dataset"
    frameLoader = FrameLoader(dataset_dirs, target_dir)
    # X, y = frameLoader.extract_data()
    # frameLoader.save_dataset()
    X, y = load_dataset(dataset_dir=target_dir)

    n_components = 2
    
    # pca_tool = PCALearnedFeatures(X, n_components=n_components)
    # pca_amp, pca_ang, pca_combined = pca_tool.full_monty()

    # plot_feature_reduction(y, pca_amp, "PCA of I/Q signal amplitude", show_plot=False)
    # plot_feature_reduction(y, pca_ang, "PCA of I/Q signal angle", show_plot=False)
    # plot_feature_reduction(y, pca_combined, "PCA of I/Q signal amplitude and angle", show_plot=False)
    # plt.show()

    # kpca_tool = kPCALearnedFeatures(X, y, n_components=n_components)
    # pca_amp, pca_ang, pca_combined = kpca_tool.full_monty()

    # plot_feature_reduction(y, pca_amp, "kPCA of I/Q signal amplitude", show_plot=False)
    # plot_feature_reduction(y, pca_ang, "kPCA of I/Q signal angle", show_plot=False)
    # plot_feature_reduction(y, pca_combined, "kPCA of I/Q signal amplitude and angle", show_plot=False)
    # plt.show()

    # epochs = 1000

    # X_amp = np.abs(X)
    # autoencoder = AutoencoderLearnedFeatures(X_amp, y, epochs=epochs, batch_size=256, verbose=True)
    # encoded_amplitude = autoencoder.full_monty(X_amp)
    # X_pha = np.unwrap(np.angle(X))
    # autoencoder = AutoencoderLearnedFeatures(X_pha, y, epochs=epochs, batch_size=256, verbose=True)
    # encoded_phase = autoencoder.full_monty(X_pha)
    # X_com = np.concatenate((np.abs(X), np.unwrap(np.angle(X))), axis=1)
    # autoencoder = AutoencoderLearnedFeatures(X_com, y, epochs=epochs, batch_size=256, verbose=True)
    # encoded_combined = autoencoder.full_monty(X_com)

    # # Plot the encoded features
    # plot_feature_reduction(y, encoded_amplitude, "Autoencoder Feature Reduction (Amplitude)", show_plot=False)
    # plot_feature_reduction(y, encoded_phase, "Autoencoder Feature Reduction (Phase)", show_plot=False)
    # plot_feature_reduction(y, encoded_combined, "Autoencoder Feature Reduction (Combined)", show_plot=False)
    # plt.show()

    # Amplitude
    X_amp = np.abs(X)
    cnn_amp = CNNLearnedFeatures(X_amp, y, epochs=20, verbose=False)
    features_amp = cnn_amp.full_monty(X_amp)
    plot_feature_reduction(y, features_amp, "CNN Feature Reduction (Amplitude)", show_plot=False)

    # Phase
    X_pha = np.unwrap(np.angle(X))
    cnn_pha = CNNLearnedFeatures(X_pha, y, epochs=20, verbose=False)
    features_pha = cnn_pha.full_monty(X_pha)
    plot_feature_reduction(y, features_pha, "CNN Feature Reduction (Phase)", show_plot=False)

    # Combined
    X_com = np.concatenate((X_amp, X_pha), axis=1)
    cnn_com = CNNLearnedFeatures(X_com, y, epochs=20, verbose=False)
    features_com = cnn_com.full_monty(X_com)
    plot_feature_reduction(y, features_com, "CNN Feature Reduction (Combined)", show_plot=True)