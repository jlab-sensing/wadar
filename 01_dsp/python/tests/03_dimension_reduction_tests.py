# Following the roadmap by 
# https://scikit-learn.org/stable/machine_learning_map.html
# to perform dimensionality reduction for the purpose
# of looking at the data in a lower dimension.

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
sys.path.insert(0, parent_dir)
from _03_hephaestus.pca_tools import PCAProcessor, KernelPCAProcessor
from _01_gaia.loader import FrameLoader
from _06_hermes.bulk_density_labels import bulk_density_to_label
from sklearn.manifold import Isomap
from sklearn.neighbors import kneighbors_graph
from sklearn.manifold import spectral_embedding
from sklearn.manifold import locally_linear_embedding
from sklearn.decomposition import PCA

if __name__ == "__main__":

    CLASSIFICATION_MODE = True

    # Load the dataset
    dataset_dir = "../data/combined-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False)
    X, y = hydros.X, hydros.y

    N, R, T = X.shape
    X_flat = X.reshape(N, R * T)
    X_flat = np.abs(X_flat)

    if CLASSIFICATION_MODE:
        y_labels = []
        for i, j in enumerate(y):
            y_labels.append(bulk_density_to_label(j, soil_type="silty"))
        y = np.array([0 if label == "Ideal" else 1 if label == "Non-ideal" else 2 for label in y_labels])
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes = axes.flatten()

    # ==============
    # PCA
    # ==============
    pca = PCA(n_components=2)
    normal_pca = pca.fit_transform(X_flat)
    sc = axes[0].scatter(normal_pca[:, 0], normal_pca[:, 1], c=y, cmap='viridis', alpha=0.5)
    axes[0].set_xlabel('PCA Component 1')
    axes[0].set_ylabel('PCA Component 2')
    axes[0].set_title('Normal PCA Dimensionality Reduction (2D)')
    fig.colorbar(sc, ax=axes[0], label='Label')

    # ==============
    # Isomap
    # ==============
    embedding = Isomap(n_components=2, n_neighbors=int(X.shape[0] / 10))
    X_transformed = embedding.fit_transform(X_flat)
    sc = axes[1].scatter(X_transformed[:, 0], X_transformed[:, 1], c=y, cmap='viridis', alpha=0.5)
    axes[1].set_xlabel('Isomap Component 1')
    axes[1].set_ylabel('Isomap Component 2')
    axes[1].set_title('Isomap Dimensionality Reduction (2D)')
    fig.colorbar(sc, ax=axes[1], label='Label')

    # ==============
    # Spectral Embedding
    # ==============
    affinity_matrix = kneighbors_graph(
        X_flat, n_neighbors=int(X.shape[0] / 10), include_self=True
    )
    affinity_matrix = 0.5 * (affinity_matrix + affinity_matrix.T)
    embedding = spectral_embedding(affinity_matrix, n_components=2, random_state=42)
    sc = axes[2].scatter(embedding[:, 0], embedding[:, 1], c=y, cmap='viridis', alpha=0.5)
    axes[2].set_xlabel('Spectral Embedding Component 1')
    axes[2].set_ylabel('Spectral Embedding Component 2')
    axes[2].set_title('Spectral Embedding Dimensionality Reduction (2D)')
    fig.colorbar(sc, ax=axes[2], label='Label')

    # ==============
    # Locally Linear Embedding
    # ==============
    embedding, _ = locally_linear_embedding(
        X_flat, n_components=2, n_neighbors=int(X.shape[0] / 10)
    )
    sc = axes[3].scatter(embedding[:, 0], embedding[:, 1], c=y, cmap='viridis', alpha=0.5)
    axes[3].set_xlabel('Locally Linear Embedding Component 1')
    axes[3].set_ylabel('Locally Linear Embedding Component 2')
    axes[3].set_title('Locally Linear Embedding Dimensionality Reduction (2D)')
    fig.colorbar(sc, ax=axes[3], label='Label')

    plt.tight_layout()

    # Now using the angle instead of the absolute value
    X_flat = X.reshape(N, R * T)
    X_flat = np.angle(X_flat)

    fig2, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes = axes.flatten()

    # ==============
    # PCA
    # ==============
    pca = PCA(n_components=2)
    normal_pca = pca.fit_transform(X_flat)
    sc = axes[0].scatter(normal_pca[:, 0], normal_pca[:, 1], c=y, cmap='viridis', alpha=0.5)
    axes[0].set_xlabel('PCA Component 1')
    axes[0].set_ylabel('PCA Component 2')
    axes[0].set_title('Normal PCA Dimensionality Reduction (2D)')
    fig2.colorbar(sc, ax=axes[0], label='Label')

    # ==============
    # Isomap
    # ==============
    embedding = Isomap(n_components=2, n_neighbors=int(X.shape[0] / 10))
    X_transformed = embedding.fit_transform(X_flat)
    sc = axes[1].scatter(X_transformed[:, 0], X_transformed[:, 1], c=y, cmap='viridis', alpha=0.5)
    axes[1].set_xlabel('Isomap Component 1')
    axes[1].set_ylabel('Isomap Component 2')
    axes[1].set_title('Isomap Dimensionality Reduction (2D)')
    fig2.colorbar(sc, ax=axes[1], label='Label')

    # ==============
    # Spectral Embedding
    # ==============
    affinity_matrix = kneighbors_graph(
        X_flat, n_neighbors=int(X.shape[0] / 10), include_self=True
    )
    affinity_matrix = 0.5 * (affinity_matrix + affinity_matrix.T)
    embedding = spectral_embedding(affinity_matrix, n_components=2, random_state=42)
    sc = axes[2].scatter(embedding[:, 0], embedding[:, 1], c=y, cmap='viridis', alpha=0.5)
    axes[2].set_xlabel('Spectral Embedding Component 1')
    axes[2].set_ylabel('Spectral Embedding Component 2')
    axes[2].set_title('Spectral Embedding Dimensionality Reduction (2D)')
    fig2.colorbar(sc, ax=axes[2], label='Label')

    # ==============
    # Locally Linear Embedding
    # ==============
    embedding, _ = locally_linear_embedding(
        X_flat, n_components=2, n_neighbors=int(X.shape[0] / 10)
    )
    sc = axes[3].scatter(embedding[:, 0], embedding[:, 1], c=y, cmap='viridis', alpha=0.5)
    axes[3].set_xlabel('Locally Linear Embedding Component 1')
    axes[3].set_ylabel('Locally Linear Embedding Component 2')
    axes[3].set_title('Locally Linear Embedding Dimensionality Reduction (2D)')
    fig2.colorbar(sc, ax=axes[3], label='Label')

    plt.tight_layout()

    plt.show()
