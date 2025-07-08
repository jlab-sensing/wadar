import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _03_hephaestus.pca_tools import PCAProcessor
from _01_gaia.loader import FrameLoader

if __name__ == "__main__":

    # Load the dataset
    dataset_dir = "../data/dry-bulk-density-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False)
    X, y = hydros.X, hydros.y

    # ==============
    # Using PCAProcessor to perform dimensionality reduction
    hephaestus = PCAProcessor(X, n_components=3)
    temp = hephaestus.dimensionality_reduction()

    for i, j in enumerate(y):
        if y[i] < 1.4:
            y[i] = 0
        else:
            y[i] = 1

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(temp[:, 0], temp[:, 1], temp[:, 2], c=y, cmap='viridis', alpha=0.5)
    ax.set_xlabel('PCA Component 1')
    ax.set_ylabel('PCA Component 2')
    ax.set_zlabel('PCA Component 3')
    ax.set_title('3D PCA Dimensionality Reduction')
    plt.colorbar(ax.scatter(temp[:, 0], temp[:, 1], temp[:, 2], c=y, cmap='viridis', alpha=0.5), label='Label')
    plt.show()

    # ==============
    # Using PCAProcessor to denoise the data
    back_to_original_shape = hephaestus.denoise()

    X = np.abs(X)
    plt.plot(np.median(back_to_original_shape[3, :, :], axis=1), label='Denoised Frame 0')
    plt.plot(np.median(X[3, :, :], axis=1), label='Original Frame 0')
    plt.legend()
    plt.show()

    # ==============
    # Plot variance explained by PCA components
    # hephaestus.plot_variance_columnwise()

    # ==============
    # Plot variance explained by PCA components per sample
    # hephaestus.plot_variance_samplewise()