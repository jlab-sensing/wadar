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
    best_models_plot = best_models[['Feature', 'Model', 'MAE', 'Accuracy']].copy()
    # Combine feature and model for x-axis labels, abbreviate if too long
    def abbreviate(text, maxlen=12):
        return text if len(text) <= maxlen else text[:maxlen-3] + '...'
    best_models_plot['Name'] = best_models_plot['Feature'].apply(abbreviate) + '→' + best_models_plot['Model'].apply(abbreviate)
    names = best_models_plot['Name']
    mae = best_models_plot['MAE']
    accuracy = best_models_plot['Accuracy']
    index = np.arange(len(names))
    bar_width = 0.35

    fig, ax1 = plt.subplots(figsize=(max(8, len(names)*0.8), 6))

    # Plot MAE bars
    mae_bars = ax1.bar(index, mae, bar_width, label='MAE', color='#1f77b4', edgecolor='black', linewidth=1)
    # Plot Accuracy bars (secondary axis)
    ax2 = ax1.twinx()
    acc_bars = ax2.bar(index + bar_width, accuracy, bar_width, label='Accuracy', color='#ff7f0e', edgecolor='black', linewidth=1)

    # Axis labels and ticks
    ax1.set_xlabel('Feature→Model', fontsize=14, fontweight='bold')
    ax1.set_ylabel('MAE', color='#1f77b4', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Accuracy', color='#ff7f0e', fontsize=14, fontweight='bold')
    ax1.set_xticks(index + bar_width / 2)
    ax1.set_xticklabels(names, rotation=30, ha='right', fontsize=12, wrap=True)

    # Add grid for MAE axis
    ax1.grid(True, axis='y', linestyle='--', alpha=0.6)

    # Add value labels on bars
    for bar in mae_bars:
        height = bar.get_height()
        ax1.annotate(f'{height:.2f}', xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3), textcoords="offset points",
                        ha='center', va='bottom', fontsize=10)
    for bar in acc_bars:
        height = bar.get_height()
        ax2.annotate(f'{height:.2f}', xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3), textcoords="offset points",
                        ha='center', va='bottom', fontsize=10)

    bars = [mae_bars, acc_bars]
    labels = ['MAE (g/cm^3)', 'Accuracy']
    ax1.legend(bars, labels, loc='best', fontsize=12)

    ax1.set_ylim(0, mae.max() * 1.5)
    ax2.set_ylim(0, accuracy.max() * 1.1)

    plt.tight_layout()
    plt.show()


def generate_results_summary(validation_dataset, results_file_name="validation_results.csv", top_n=10):
    """
    Generate and display results summary with visualization.
    
    Args:
        validation_dataset: Path to validation dataset
        top_n: Number of top models to show
    """
    print("[INFO] Generating results summary...")
    
    try:
        results = load_results(validation_dataset, results_file_name)
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

def visualize_dimensionality_reduction(X_train_reduced, y_train, X_val_reduced, y_val, method='Autoencoder', title=None):
    """
    Generic function to visualize already-reduced dimensionality data.
    
    Args:
        X_train_reduced: Already reduced training features (2D array)
        y_train: Training labels (1D array)
        X_val_reduced: Already reduced validation features (2D array)
        y_val: Validation labels (1D array)
        method: Name of the reduction method (for labeling)
        title: Optional plot title
    """
    # Take first 2 dimensions if more than 2 are available
    if X_train_reduced.shape[1] > 2:
        X_train_plot = X_train_reduced[:, :2]
        X_val_plot = X_val_reduced[:, :2]
    else:
        X_train_plot = X_train_reduced
        X_val_plot = X_val_reduced
    
    # Create plot
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Training set: circles
    scatter_train = ax.scatter(
        X_train_plot[:, 0], X_train_plot[:, 1],
        c=y_train, cmap='tab10', alpha=0.7,
        edgecolors='black', linewidth=0.5,
        marker='o', s=60, label='Training'
    )
    
    # Validation set: triangles
    scatter_val = ax.scatter(
        X_val_plot[:, 0], X_val_plot[:, 1],
        c=y_val, cmap='tab10', alpha=0.8,
        edgecolors='black', linewidth=0.5,
        marker='^', s=80, label='Validation'
    )
    
    # Styling
    ax.set_xlabel(f'{method} Component 1', fontsize=12)
    ax.set_ylabel(f'{method} Component 2', fontsize=12)
    
    if title is None:
        title = f'{method} Visualization: Training vs Validation'
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=11)
    
    # Add colorbar
    cbar = plt.colorbar(scatter_train, ax=ax, label='Labels')
    cbar.set_label('Labels', fontsize=11)
    
    plt.tight_layout()
    plt.show()