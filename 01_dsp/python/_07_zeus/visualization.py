"""
Visualization Utilities for Zeus Pipeline
Contains functions for plotting and displaying results.
"""

import numpy as np
import matplotlib.pyplot as plt
from _06_hermes.logger import load_results
from sklearn.manifold import TSNE


def plot_results(top_n, results):
    """
    Plot and display top performing models.
    
    Args:
        top_n: Number of top models to display
        results: DataFrame with model results
    """
    if not results.empty:
        sorted_results = results.sort_values(by='MAE', ascending=True)
        best_models = sorted_results.head(top_n)
        print(f"\n[INFO] Top {top_n} Best Performing Models (Lowest MAE):")
        print(best_models[['Feature', 'Model', 'Accuracy', 'MAE', 'R2', 'RMSE']].to_string(index=False))
    else:
        print("[INFO] No training results found.")
        return

    # Prepare data for plotting
    best_models_plot = best_models[['Feature', 'Model', 'MAE']].copy()
    features = best_models_plot['Feature'].unique()
    models = best_models_plot['Model'].unique()

    # Create a pivot table for MAE
    mae_pivot = best_models_plot.pivot(index='Feature', columns='Model', values='MAE')

    x = np.arange(len(features))  # the label locations
    width = 0.7  # the width of the bars
    n_models = len(models)
    bar_width = width / n_models

    fig, ax = plt.subplots(figsize=(6, 3))  # IEEE column width

    # Plot MAE bars
    for idx, model in enumerate(models):
        offsets = (idx - (n_models-1)/2) * bar_width
        ax.bar(x + offsets, mae_pivot[model], bar_width, label=f"{model}")

    ax.set_ylabel('MAE', fontsize=8)
    ax.set_title('Top Models by Feature: MAE', fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(features, rotation=45, ha='right', fontsize=7)
    ax.tick_params(axis='y', labelsize=7)
    ax.grid(axis='y', linestyle='--', alpha=0.5, linewidth=0.5)
    ax.legend(title='Model', loc='upper right', fontsize=7, title_fontsize=7, frameon=False)

    plt.tight_layout(pad=0.5)
    plt.subplots_adjust(bottom=0.25)
    plt.show()


def generate_results_summary(validation_dataset, top_n=10):
    """
    Generate and display results summary with visualization.
    
    Args:
        validation_dataset: Path to validation dataset
        top_n: Number of top models to show
    """
    print("[INFO] Generating results summary...")
    
    try:
        results = load_results(validation_dataset, "validation_results.csv")
        plot_results(top_n, results)
    except Exception as e:
        print(f"[ERROR] Failed to load or plot results: {e}")
        print("[INFO] No results available for plotting.")

def plot_reduced_features(reduced_training_features, reduced_validation_features, labels_train, labels_val, title):
    """
    Visualize dimensionality-reduced features for training and validation sets.

    Args:
        reduced_training_features_combined: np.ndarray, shape (n_samples_train, 2)
        reduced_validation_features_combined: np.ndarray, shape (n_samples_val, 2)
        labels_train: array-like, labels for training samples
        labels_val: array-like, labels for validation samples
    """
    fig, ax = plt.subplots(figsize=(8, 6))

    # Training set: circles
    scatter_train = ax.scatter(
        reduced_training_features[:, 0],
        reduced_training_features[:, 1],
        c=labels_train,
        cmap='tab10', alpha=0.7, edgecolors='k', marker='o', label='Train'
    )
    # Validation set: larger triangles, use same colormap and normalization
    scatter_val = ax.scatter(
        reduced_validation_features[:, 0],
        reduced_validation_features[:, 1],
        c=labels_val,
        cmap=scatter_train.cmap,
        norm=scatter_train.norm,
        alpha=0.7, edgecolors='k', marker='^', label='Validation',
        s=120
    )

    ax.set_title(f"{title} Top 2 Components: Train (o) vs Validation (^)")
    ax.legend(loc='best')

    # Colorbar for labels (only one, since cmap and norm are shared)
    cbar = plt.colorbar(scatter_train, ax=ax, label='Label')
    plt.tight_layout()

def tsne_plot(reduced_training_features, reduced_validation_features, labels_train, labels_val, random_state=42):
    """
    Apply t-SNE and visualize reduced features for training and validation sets.

    Args:
        reduced_training_features: np.ndarray, shape (n_samples_train, n_features)
        reduced_validation_features: np.ndarray, shape (n_samples_val, n_features)
        labels_train: array-like, labels for training samples
        labels_val: array-like, labels for validation samples
        random_state: int, random seed for reproducibility
    """
    # Concatenate features for joint t-SNE
    features_combined = np.vstack([reduced_training_features, reduced_validation_features])
    labels_combined = np.concatenate([labels_train, labels_val])
    n_train = reduced_training_features.shape[0]

    # Apply t-SNE
    tsne = TSNE(n_components=2, random_state=random_state)
    features_tsne = tsne.fit_transform(features_combined)

    # Split back to train/val
    train_tsne = features_tsne[:n_train]
    val_tsne = features_tsne[n_train:]

    fig, ax = plt.subplots(figsize=(8, 6))

    scatter_train = ax.scatter(
        train_tsne[:, 0], train_tsne[:, 1],
        c=labels_train, cmap='tab10', alpha=0.7, edgecolors='k', marker='o', label='Train'
    )
    scatter_val = ax.scatter(
        val_tsne[:, 0], val_tsne[:, 1],
        c=labels_val, cmap=scatter_train.cmap, norm=scatter_train.norm,
        alpha=0.7, edgecolors='k', marker='^', label='Validation', s=120
    )

    ax.set_title("t-SNE: Train (o) vs Validation (^)")
    ax.legend(loc='best')
    plt.colorbar(scatter_train, ax=ax, label='Label')
    plt.tight_layout()