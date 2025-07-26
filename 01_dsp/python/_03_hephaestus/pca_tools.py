import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from sklearn.decomposition import PCA

class PCAProcessor:
    """
    PCAProcessor is a class to perform Principal Component Analysis (PCA) on radar data.
    It can be used to reduce the dimensionality of the data and extract features that are useful
    for classification tasks.

    Parameters:
        X (np.ndarray):         Input data of shape (samples, features).
        n_components (int):     Number of components to keep.
    """

    def __init__(self, X, n_components=None):
        """
        Initialize the PCAProcessor with the input data and number of components.

        Args:
            X (np.ndarray):         Input data of shape (samples, features).
            n_components (int):     Number of components to keep.
        """

        self.n_components = n_components
        self.pca = PCA(n_components=self.n_components)
        self.X = X

        if np.iscomplex(X).any():
            raise ValueError("Input the magnitude or phase not the raw data.")
    
    def dimensionality_reduction(self, X=None):
        """
        Perform PCA on the input data and return the reduced data.

        Returns:
            np.ndarray: Reduced data of shape (samples, n_components).
        """

        if X is None:
            X = self.X
        N, R, T = X.shape
        X_flat = X.reshape(N, R * T)
        reduced = self.pca.fit_transform(X_flat)  # shape: (N, n_components)
        return reduced

    def plot_variance_samplewise(self, max_components=10):
        """
        Plot the cumulative variance explained by the PCA components for each sample. Useful for understanding how many components are needed
        to explain the variance in the data.

        Args:
            max_components (int): Maximum number of components to consider for the plot.
        """
        N, R, T = self.X.shape
        X_flat = np.abs(self.X.reshape(N, R * T))  # flatten per sample
        explained = []
        for n in range(1, max_components + 1):
            pca = PCA(n_components=n)
            pca.fit(X_flat)
            explained.append(np.sum(pca.explained_variance_ratio_))
        plt.plot(range(1, max_components + 1), explained, marker='o')
        plt.xlabel('Number of PCA Components')
        plt.ylabel('Cumulative Variance Explained')
        plt.title('Variance Explained (Per-Sample Dimensionality Reduction)')
        plt.grid()
        plt.show()