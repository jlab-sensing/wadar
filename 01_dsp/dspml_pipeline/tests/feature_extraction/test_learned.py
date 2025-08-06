import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.feature_extraction.learned.pca import PCAProcessor
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
    
    pca_tool = PCAProcessor(X, n_components=n_components)
    pca_amp, pca_ang, pca_combined = pca_tool.dimensionality_reduction()

    plot_feature_reduction(y, pca_amp, "PCA of I/Q signal amplitude", show_plot=False)
    plot_feature_reduction(y, pca_ang, "PCA of I/Q signal angle", show_plot=False)
    plot_feature_reduction(y, pca_combined, "PCA of I/Q signal amplitude and angle", show_plot=False)
    plt.show()