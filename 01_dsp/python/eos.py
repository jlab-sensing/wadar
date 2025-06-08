# Eos (Ἠώς) is the Greek goddess of dawn, symbolizing new beginnings. Here, it represents the dawn of data processing, where
# light and clarity are brought to the raw radar frames.

import numpy as np

import pandas as pd  
from sklearn.preprocessing import StandardScaler  
from sklearn.decomposition import PCA 

import matplotlib.pyplot as plt  
from hydros import HydrosFrameLoader  

class EosDenoising:
    def __init__(self, n_components=1):
        self.n_components = n_components
        self.scaler = StandardScaler()
        self.pca = PCA(n_components=self.n_components)
        self.X_shape = None

    def fit(self, X):
        self.X_shape = X.shape
        N, R, T = self.X_shape
        X_flat = X.reshape(N, R * T)
        X_scaled = self.scaler.fit_transform(X_flat)
        self.pca.fit(X_scaled)

    def transform(self, X):
        N, R, T = X.shape
        X_flat = X.reshape(N, R * T)
        X_scaled = self.scaler.transform(X_flat)

        pcs = self.pca.transform(X_scaled)
        reconstructed = self.pca.inverse_transform(pcs)
        cleaned_flat = X_scaled - reconstructed

        cleaned = cleaned_flat.reshape(N, R, T)
        return cleaned

    def clean_frames(self, X):
        self.fit(X)
        return self.transform(X)

    def plot_variance(self, X, max_components=10):
        X_flat = X.reshape(X.shape[0], -1)
        X_scaled = self.scaler.fit_transform(X_flat)
        nums = np.arange(1, max_components + 1)
        var_ratio = []
        for num in nums:
            pca = PCA(n_components=num)
            pca.fit(X_scaled)
            var_ratio.append(pca.explained_variance_ratio_.sum())
            print(f"n_components: {num}, Explained Variance Ratio: {pca.explained_variance_ratio_.sum()}")
        plt.figure()
        plt.grid()
        plt.plot(nums, var_ratio, marker='o')
        plt.xlabel('n_components')
        plt.ylabel('Unexplained variance ratio')
        plt.xticks(nums)
        plt.show()

def viz_noise_removal(X, cleaned, y):
    unique_Y, unique_Y_idx = np.unique(y, return_index=True)
    plt.figure(figsize=(8, 2.5 * len(unique_Y)))
    for i, j in enumerate(unique_Y_idx):
        # Show cleaned image
        plt.subplot(len(unique_Y), 2, i * 2 + 1)
        plt.plot(cleaned[j, :, :])
        plt.title(f"Cleaned (Label: {unique_Y[i]:.2f})")
        plt.axis('off')
        # Show original image
        plt.subplot(len(unique_Y), 2, i * 2 + 2)
        plt.plot(X[j, :, :])
        plt.title(f"Original (Label: {unique_Y[i]:.2f})")
        plt.axis('off')
    plt.tight_layout()
    plt.show()

# ===========================
# Test Harness
# ===========================
if __name__ == "__main__":
    hydros = HydrosFrameLoader("data/test_set")
    X, y = hydros.load_raw_data()
    eos = EosDenoising(n_components=2)

    # reducer.plot_variance(X, max_components=10)

    cleaned = eos.clean_frames(X)
    viz_noise_removal(X, cleaned, y)