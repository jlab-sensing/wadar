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

    def fit_global(self, X):
        self.X_shape = X.shape
        N, R, T = self.X_shape
        X_flat = X.reshape(N, R * T)
        X_scaled = self.scaler.fit_transform(X_flat)
        self.pca.fit(X_scaled)

    def transform_global(self, X):
        N, R, T = X.shape
        X_flat = X.reshape(N, R * T)
        X_scaled = self.scaler.transform(X_flat)

        pcs = self.pca.transform(X_scaled)
        reconstructed = self.pca.inverse_transform(pcs)
        cleaned_flat = X_scaled - reconstructed

        cleaned = cleaned_flat.reshape(N, R, T)
        return cleaned

    def global_pca_denoise(self, X):
        self.fit_global(X)
        return self.transform_global(X)

    def fit_local(self, X):
        N, R, T = X.shape
        self.local_models = []

        for i in range(N):
            frame = X[i].T  
            scaler = StandardScaler()
            frame_scaled = scaler.fit_transform(frame)

            pca = PCA(n_components=self.n_components)
            pca.fit(frame_scaled)

            self.local_models.append((scaler, pca))

    def transform_local(self, X):
        N, R, T = X.shape
        cleaned = np.zeros_like(X)

        for i in range(N):
            frame = X[i].T
            scaler, pca = self.local_models[i]

            frame_scaled = scaler.transform(frame)
            pcs = pca.transform(frame_scaled)
            reconstructed = pca.inverse_transform(pcs)
            denoised = frame_scaled - reconstructed 
            cleaned[i] = denoised.T 

        return cleaned

    def local_pca_denoise(self, X):
        self.fit_local(X)
        return self.transform_local(X)

    def plot_variance(self, X, max_components=10):
        X_flat = np.abs(X.reshape(X.shape[0], -1))
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
        # plt.plot(cleaned[j, :, :])
        plt.plot(np.median(cleaned[j], axis=1))
        plt.title(f"Cleaned (Label: {unique_Y[i]:.2f})")
        plt.axis('off')
        # Show original image
        plt.subplot(len(unique_Y), 2, i * 2 + 2)
        # plt.plot(X[j, :, :])
        plt.plot(np.median(X[j], axis=1))
        plt.title(f"Original (Label: {unique_Y[i]:.2f})")
        plt.axis('off')
    plt.tight_layout()
    plt.show()

# ===========================
# Test Harness
# ===========================
if __name__ == "__main__":
    hydros = HydrosFrameLoader("data/compact-4-dry", new_dataset=False)
    X, y = hydros.X, hydros.y
    
    eos = EosDenoising(n_components=2)

    eos.plot_variance(X, max_components=10)

    # # cleaned = eos.global_pca_denoise(X)
    # # hydros.update_dataset(cleaned, y)
    # # viz_noise_removal(X.T, cleaned, y)

    # cleaned = eos.local_pca_denoise(X)
    # hydros.update_dataset(cleaned, y)
    # viz_noise_removal(X, cleaned, y)