import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.set_loglevel("warning") 

import pandas as pd

def plot_IQ_signals(X:np.ndarray, y:np.ndarray, show_plot:bool = True):
    """
    Plot the mean and standard deviation of the absolute value and angle of the I/Q signals using a colorbar.

    Args:
        X (np.ndarray):     Input data of shape (samples, range_bins, time_bins)
        y (np.ndarray):     Labels of shape (samples,)
        show_plot (bool):   Determines whether plt.show() is called at the end
    """

    # Set font for publication-quality figures
    plt.rcParams.update({
        "font.size": 16,
        "font.family": "serif",
        "axes.labelsize": 18,
        "axes.titlesize": 20,
        "xtick.labelsize": 15,
        "ytick.labelsize": 15,
        "legend.fontsize": 15,
        "figure.titlesize": 22
    })

    unique_Y = np.unique(y)

    # ===== Plotting amplitude =====

    X_abs = np.median(np.abs(X), axis=2)

    fig, ax = plt.subplots(figsize=(12, 7))

    sorted_unique_Y = np.sort(unique_Y)
    norm = mpl.colors.Normalize(vmin=sorted_unique_Y.min(), vmax=sorted_unique_Y.max())
    cmap = plt.cm.inferno

    for label in sorted_unique_Y:
        idx = np.where(y == label)[0]
        color = cmap(norm(label))

        mean_curve = np.mean(X_abs[idx], axis=0)
        std_curve = np.std(X_abs[idx], axis=0)
        ax.plot(mean_curve, color=color, linewidth=2)
        ax.fill_between(np.arange(mean_curve.size), mean_curve - std_curve, mean_curve + std_curve,
                        color=color, alpha=0.2)
    
    ax.set_xlabel("Range Bin")
    ax.set_ylabel("Amplitude of I/Q Signal")
    ax.grid(True, linestyle='--', alpha=0.5)
    
    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label("Compaction Level")
    
    plt.tight_layout()

    # ===== Plotting angle =====

    X_angle = np.median(np.angle(X), axis=2)
    X_angle = np.unwrap(X_angle)

    fig, ax = plt.subplots(figsize=(12, 7))

    for label in sorted_unique_Y:
        idx = np.where(y == label)[0]
        color = cmap(norm(label))

        mean_curve = np.mean(X_angle[idx], axis=0)
        std_curve = np.std(X_angle[idx], axis=0)
        ax.plot(mean_curve, color=color, linewidth=2)
        ax.fill_between(np.arange(mean_curve.size), mean_curve - std_curve, mean_curve + std_curve,
                        color=color, alpha=0.2)
    
    ax.set_xlabel("Range Bin")
    ax.set_ylabel("Phase of I/Q Signal (radians)")
    ax.grid(True, linestyle='--', alpha=0.5)
    
    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label("Compaction Level")
    
    plt.tight_layout()

    if show_plot:
        plt.show()

def plot_feature_reduction(labels:np.ndarray, feature_array:np.ndarray, figure_title:str, show_plot:bool=True):
    """
    Plots a 2D scatter of feature-reduced data colored by labels.

    Args:
        labels (np.ndarray): Array of labels for each sample.
        feature_array (np.ndarray): 2D array of reduced features (samples, 2).
        figure_title (str): Title for the plot.
        show_plot (bool): Whether to display the plot immediately.
    """

    plt.figure(figsize=(8, 6))
    scatter = plt.scatter(feature_array[:, 0], feature_array[:, 1], c=labels, cmap='inferno', edgecolor='k', s=60)
    plt.colorbar(scatter, label='Bulk density (g/cm^3)')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.title(figure_title)
    plt.tight_layout()

    if show_plot:
        plt.show()

def plot_accuracy_mae(results:pd.DataFrame, figure_title:str, show_plot:bool = True):
    """
    Plots a grouped bar chart of MAE and Accuracy for each model. Generated almost entirely
    by copilot.

    Args:
        results (pd.DataFrame): DataFrame containing 'Model', 'MAE', and 'Accuracy' columns.
        figure_title (str): Title for the plot.
        show_plot (bool): Whether to display the plot immediately.
    """

    fig, ax1 = plt.subplots(figsize=(10, 6))

    # Sort by Accuracy descending
    sorted_results = results.sort_values(by='Accuracy', ascending=False)
    models = sorted_results['Model']
    mae = sorted_results['MAE']
    accuracy = sorted_results['Accuracy']
    index = np.arange(len(models))

    bar_width = 0.35

    # Use inferno colormap for both bars
    cmap = plt.cm.inferno
    colors = [cmap(0.3), cmap(0.7)]  # Two distinct inferno colors

    # Plot MAE
    mae_bars = ax1.bar(index, mae, bar_width, label='MAE', color=colors[0], edgecolor='black', linewidth=1)
    # Plot Accuracy
    ax2 = ax1.twinx()
    acc_bars = ax2.bar(index + bar_width, accuracy, bar_width, label='Accuracy', color=colors[1], edgecolor='black', linewidth=1)

    # Axis labels and ticks
    ax1.set_xlabel('Model', fontsize=14, fontweight='bold')
    ax1.set_ylabel('MAE', color=colors[0], fontsize=14, fontweight='bold')
    ax2.set_ylabel('Accuracy', color=colors[1], fontsize=14, fontweight='bold')
    ax1.set_xticks(index + bar_width / 2)
    ax1.set_xticklabels(models, rotation=45, ha='right', fontsize=12)

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

    plt.title(f"Features: {figure_title}")

    plt.tight_layout()
    if show_plot:
        plt.show()