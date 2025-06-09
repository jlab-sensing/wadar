# Hephaestus (Ἥφαιστος): God of blacksmiths, metalworking, and craftsmanship. This file sses functions for shaping, forging, and 
# transforming data into features.

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from hydros import HydrosFrameLoader
from eos import EosDenoising
import os
from scipy import signal
from sklearn.decomposition import PCA
from mpl_toolkits.mplot3d import Axes3D

class HephaestusPCA:
    def __init__(self, X, n_components=1):
        self.n_components = n_components
        self.pca = PCA(n_components=self.n_components)
        self.X = X
    
    def denoise(self):
        new_shape = self.shape_data2pca()
        clutter = self.pca.fit_transform(new_shape)
        clutter_recon = self.pca.inverse_transform(clutter)
        denoised_new_shape = new_shape - clutter_recon
        return self.shape_pca2data(denoised_new_shape)

    def shape_data2pca(self):
        N, R, T = self.X.shape
        new_shape = np.zeros((R, T * N), dtype=np.float64)
        new_shape_idx = 0
        for i in range(N):
            for j in range(T):
                new_shape[:, new_shape_idx] = np.abs(self.X[i, :, j])
                new_shape_idx += 1
        return new_shape

    def shape_pca2data(self, new_shape):
        N, R, T = self.X.shape
        back_to_original_shape = np.zeros_like(self.X, dtype=np.float64)
        for i in range(N):
            for j in range(T):
                back_to_original_shape[i, :, j] = new_shape[:, i * T + j]
        return back_to_original_shape
    
    def dimensionality_reduction(self):
        N, R, T = self.X.shape
        X_flat = self.X.reshape(N, R * T)
        X_flat = np.abs(X_flat)
        reduced = self.pca.fit_transform(X_flat)  # shape: (N, n_components)
        print("Shape after PCA dimensionality reduction:", reduced.shape)
        return reduced
    
    def plot_variance_columnwise(self):
        new_shape = self.shape_data2pca()
        pca_components = range(1, 11) 
        var_explained = []
        for n_components in pca_components:
            pca = PCA(n_components=n_components)
            pca.fit(new_shape)
            var_explained.append(np.sum(pca.explained_variance_ratio_))
        plt.plot(pca_components, var_explained, marker='o')
        plt.xlabel('Number of PCA Components')
        plt.ylabel('Variance Explained')
        plt.title('Variance Explained by PCA Components')
        plt.xticks(pca_components)
        plt.grid()
        plt.show()

    def plot_variance_samplewise(self, max_components=10):
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

if __name__ == "__main__":

    dataset_dir = "data/compact-4-dry"
    hydros = HydrosFrameLoader(dataset_dir, new_dataset=False)
    X, y = hydros.X, hydros.y

    hephaestus = HephaestusPCA(X, n_components=3)
    temp = hephaestus.dimensionality_reduction()

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(temp[:, 0], temp[:, 1], temp[:, 2], c=y, cmap='viridis', alpha=0.5)
    ax.set_xlabel('PCA Component 1')
    ax.set_ylabel('PCA Component 2')
    ax.set_zlabel('PCA Component 3')
    ax.set_title('3D PCA Dimensionality Reduction')
    plt.colorbar(ax.scatter(temp[:, 0], temp[:, 1], temp[:, 2], c=y, cmap='viridis', alpha=0.5), label='Label')
    plt.show()

    # plt.figure(figsize=(10, 5))
    # plt.scatter(temp[:, 0], temp[:, 1], c=y, cmap='viridis', alpha=0.5)
    # plt.colorbar(label='Label')
    # plt.xlabel('PCA Component 1')
    # plt.ylabel('PCA Component 2')
    # plt.title('PCA Dimensionality Reduction')
    # plt.show()

    # back_to_original_shape = hephaestus.pca_denoise()

    # X = np.abs(X)
    # plt.plot(np.median(back_to_original_shape[3, :, :], axis=1), label='Denoised Frame 0')
    # plt.plot(np.median(X[3, :, :], axis=1), label='Original Frame 0')
    # plt.legend()
    # plt.show()

    # hephaestus.plot_variance_columnwise()