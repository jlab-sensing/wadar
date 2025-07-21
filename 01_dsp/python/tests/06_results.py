import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

import pandas as pd

if __name__ == "__main__":
    
    dataset_dir = "../data/combined-soil-compaction-dataset"
    results_file = os.path.join(dataset_dir, "results.csv")

    # Load the results
    results = pd.read_csv(results_file)

    # Plot Accuracy and MAE
    fig, ax1 = plt.subplots(figsize=(10, 6))

    models = results['Model']
    mae = results['MAE']
    accuracy = results['Accuracy']

    bar_width = 0.35
    index = np.arange(len(models))

    # Both of these plots are CoPilot generated, if that wasn't obvious.

    # ==========================
    # MAE and Accuracy
    # ==========================

    # Sort by Accuracy descending
    sorted_results = results.sort_values(by='Accuracy', ascending=False)
    models = sorted_results['Model']
    mae = sorted_results['MAE']
    accuracy = sorted_results['Accuracy']
    index = np.arange(len(models))

    # Plot MAE
    mae_bars = ax1.bar(index, mae, bar_width, label='MAE', color='#1f77b4', edgecolor='black', linewidth=1)
    # Plot Accuracy
    ax2 = ax1.twinx()
    acc_bars = ax2.bar(index + bar_width, accuracy, bar_width, label='Accuracy', color='#ff7f0e', edgecolor='black', linewidth=1)

    # Axis labels and ticks
    ax1.set_xlabel('Model', fontsize=14, fontweight='bold')
    ax1.set_ylabel('MAE', color='#1f77b4', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Accuracy', color='#ff7f0e', fontsize=14, fontweight='bold')
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

    # top is getting cut off
    ax1.set_ylim(0, mae.max() * 1.1) 
    ax2.set_ylim(0, accuracy.max() * 1.1)  

    # Title
    plt.title('Model Error and Accuracy', fontsize=16, fontweight='bold')
    plt.tight_layout()

    # ==========================
    # Plot Inference Time
    # ==========================

    # Sort by Inference Time ascending
    sorted_results = results.sort_values(by='Inference Time', ascending=True)
    inferenceTime = sorted_results['Inference Time'] * 1000  # Convert to milliseconds

    plt.figure(figsize=(10, 6))
    bars = plt.bar(models, inferenceTime, color='#2ca02c', edgecolor='black', linewidth=1)
    plt.xlabel('Model', fontsize=14, fontweight='bold')
    plt.ylabel('Inference Time (ms)', fontsize=14, fontweight='bold')
    plt.title('Model Inference Time', fontsize=16, fontweight='bold')
    plt.yscale('log')
    plt.xticks(rotation=45, ha='right', fontsize=12)
    plt.grid(True, axis='y', linestyle='--', alpha=0.6)
    # Add value labels on bars
    for bar in bars:
        height = bar.get_height()
        plt.annotate(f'{height:.2e}', xy=(bar.get_x() + bar.get_width() / 2, height),
                     xytext=(0, 3), textcoords="offset points",
                     ha='center', va='bottom', fontsize=10)
    plt.ylim(0, inferenceTime.max() * 2)  # Set y-limits for Inference Time   
    plt.tight_layout()
    plt.show()