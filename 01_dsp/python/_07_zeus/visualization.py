"""
Visualization Utilities for Zeus Pipeline
Contains functions for plotting and displaying results.
"""

import numpy as np
import matplotlib.pyplot as plt
from _06_hermes.logger import load_results


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
