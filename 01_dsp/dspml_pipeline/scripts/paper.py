import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.plotting_tools.data_plotting import plot_IQ_signals

from graphviz import Digraph
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def main():

    setup_logging(verbose=False)

    # Lab data visualization
    # visualize_lab_data()

    # Flow chart of pipeline
    # dspml_pipeline()

    # Plot of our radar vs other radars in literature
    # visualize_previous_work_gpr()

    # Results plot
    # visualize_final_results()

    # Visualize RMSE with range resolution
    visualize_range_resolution_sim()

# ================================
# Helper functions for cleanliness
# ================================

def visualize_lab_data():
    dataset_dirs = ["../data/wet-0-soil-compaction-dataset",
                    "../data/wet-1-soil-compaction-dataset",
                    "../data/wet-2-soil-compaction-dataset"]
    target_dir = "../data/lab-dataset"
    frameLoader = FrameLoader(dataset_dirs, target_dir)
    X, y = frameLoader.extract_data()
    frameLoader.save_dataset()
    X, y = load_dataset(dataset_dir=target_dir)
    plot_IQ_signals(X, y)

def dspml_pipeline():
    dot = Digraph(comment='Radar Data Processing Pipeline', format='pdf')

    # Set graph attributes for a cleaner, publication-ready look
    dot.attr(rankdir='LR', size='8,5', fontsize='14', fontname='Times-Roman')
    dot.attr('node', shape='box', style='filled', color='lightgrey', fontname='Times-Roman', fontsize='12')

    # Nodes
    dot.node('A', 'Raw Radar Data\n(Fast Time × Slow Time)', shape='box', style='filled,bold', color='lightblue')

    dot.node('B1', 'Handcrafted Features', shape='box')
    dot.node('B2', 'Classical Dimensionality Reduction', shape='box')
    dot.node('B3', 'Deep Learning Feature Extraction', shape='box')

    dot.node('C1', 'Classical Regression Models', shape='box')
    dot.node('C2', 'Deep Learning Regression Models', shape='box')
    dot.node('C3', 'End-to-End Deep Learning Models', shape='box')

    dot.node('D1', 'Bulk Density\n(g/cm³)', shape='box', style='filled,bold', color='lightyellow')
    dot.node('E1', 'Compaction Classification', shape='box', style='filled,bold', color='lightgreen')

    # Edges
    dot.edge('A', 'B1')
    dot.edge('A', 'B2')
    dot.edge('A', 'B3')

    dot.edge('B1', 'C1')
    dot.edge('B2', 'C1')
    dot.edge('B3', 'C1')
    dot.edge('B1', 'C2')
    dot.edge('B2', 'C2')
    dot.edge('B3', 'C2')
    dot.edge('A', 'C3')

    dot.edge('C1', 'D1')
    dot.edge('C2', 'D1')
    dot.edge('C3', 'D1')
    dot.edge('D1', 'E1')

    # Subgraphs to make end-to-end sit with the right cluster
    with dot.subgraph(name='cluster_feature') as c:
        c.attr(style='dashed', label='Feature Extraction')
        c.node('B1')
        c.node('B2')
        c.node('B3')

    with dot.subgraph(name='cluster_model') as c:
        c.attr(style='dashed', label='Regression')
        c.node('C1')
        c.node('C2')
    # c.node('C3')

    dot.render('figures/flowchart', cleanup=True)

def visualize_previous_work_gpr():
    labels = ['Our Radar', 'Alzubaidi et al. 2024', 'Freeland et al. 2008', 'Wang et al. 2016']
    depth_00 = [33.0569, 41.3211, 66.1138, 33.0569]     # loam @ 0.0% moisture
    depth_20 = [0.2801, 0.3501, 0.5602, 0.2801]     # loam @ 20.0% moisture 
    depth_40 = [0.2090, 0.2612, 0.4179, 0.2090]     # loam @ 40.0% moisture

    c = 299792458  # m/s
    resolution = [0.004, c/(2*400e6), c/(2*1.6e9), 0.15]

    fig, ax = plt.subplots(figsize=(8, 6))

    for i in range(len(labels)):
        ax.scatter(depth_40[i], resolution[i], label=labels[i], s=200, marker='^')

    # ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_ylim(ax.get_ylim()[::-1])  # Reverse the y-axis (range resolution)
    ax.set_xlim(0, 1)

    ax.set_xlabel('Depth (m)', fontsize=12)
    ax.set_ylabel('Resolution (m)', fontsize=12)

    ax.grid(True, which="both", ls="--", linewidth=0.5)
    ax.legend(fontsize=9, loc='best')

    plt.tight_layout()
    plt.show()

def visualize_final_results():
    rmse_metric = 0.09

    d = {
        "Handcrafted to ridge regression": [0.08, 0.11, 0.92],
        "PCA to random forest": [0.08, 0.07, 10.74],
        "Kernel PCA to gradient boosted tree": [0.10, 0.10, 1.33],
        "Autoencoder to SVR": [0.08, 0.09, 0.40],
        "Pretrained CNN to MLP": [0.11, 0.14, 0.17],
        "End-to-end transformer": [1.36, 1.22, 142.42],
        "End-to-end LSTM": [0.09, 0.24, 84.56],
        "End-to-end pretrained CNN": [0.47, 0.4, 2798.87]
    }
    df = pd.DataFrame(data=d)

    methods = df.columns.tolist()
    rmse_in_lab = [df[method][0] for method in methods]
    rmse_in_situ = [df[method][1] for method in methods]

    x = np.arange(len(methods))
    width = 0.35

    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman"],
        "axes.titlesize": 18,
        "axes.labelsize": 16,
        "xtick.labelsize": 13,
        "ytick.labelsize": 13,
        "legend.fontsize": 13
    })

    # ze plot

    fig, ax = plt.subplots(figsize=(10, 6))

    cmap = plt.get_cmap('inferno')
    colors = [cmap(0.3), cmap(0.7)]

    bars1 = ax.bar(x - width/2, rmse_in_lab, width, label='In-Lab', color=colors[0], edgecolor='black', linewidth=1)
    bars2 = ax.bar(x + width/2, rmse_in_situ, width, label='In-Situ', color=colors[1], edgecolor='black', linewidth=1)

    # Add horizontal line for maximum desired error
    ax.axhline(rmse_metric, color='red', linestyle='--', linewidth=2, label='Max Desired RMSE (0.09)')

    ax.set_ylabel('RMSE (g/cm^3)', fontsize=16, fontname='Times New Roman')
    # ax.set_title('Comparison of RMSE for Different Methods', fontsize=18, fontname='Times New Roman', pad=15)
    ax.set_xticks(x)
    ax.set_xticklabels(methods, rotation=30, ha='right', fontsize=13, fontname='Times New Roman')
    ax.legend(fontsize=13, frameon=False)
    ax.grid(axis='y', linestyle='--', alpha=0.7, linewidth=0.8)
    ax.set_yscale('log')

    # Remove top and right spines for a cleaner look
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Annotate bars
    # for bar in bars1 + bars2:
    #     height = bar.get_height()
    #     if height > 0:
    #         ax.annotate(f'{height:.2f}',
    #                     xy=(bar.get_x() + bar.get_width() / 2, height),
    #                     xytext=(0, 3),
    #                     textcoords="offset points",
    #                     ha='center', va='bottom', fontsize=11, fontname='Times New Roman')

    plt.tight_layout()
    plt.show()

    # Ze zecond plot

    inference_time = [df[method][2] for method in methods]

    fig2, ax2 = plt.subplots(figsize=(10, 6))

    bars3 = ax2.bar(x, inference_time, width, color=cmap(0.5), edgecolor='black', linewidth=1)

    ax2.set_ylabel('Inference Time (ms)', fontsize=16, fontname='Times New Roman')
    ax2.set_xticks(x)
    ax2.set_xticklabels(methods, rotation=30, ha='right', fontsize=13, fontname='Times New Roman')
    ax2.grid(axis='y', linestyle='--', alpha=0.7, linewidth=0.8)
    ax2.set_yscale('log')

    ax2.spines['top'].set_visible(False)
    ax2.spines['right'].set_visible(False)

    plt.tight_layout()
    plt.show()

def visualize_range_resolution_sim():
    # Simulation of RMSE reductions for ridge regression on handcrafted features as range resolution is progressively reduced

    # range resolutions in meters
    resolutions = [0.004,   0.008,  0.016,  0.032,  0.064,  0.128]

    # rmse in g/cm^3
    rmse_scores = [0.08,    0.08,   0.08,   0.09,   0.11,   0.12]

    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman"],
        "axes.titlesize": 18,
        "axes.labelsize": 16,
        "xtick.labelsize": 13,
        "ytick.labelsize": 13,
        "legend.fontsize": 13
    })

    fig, ax = plt.subplots(figsize=(8, 6))

    cmap = plt.get_cmap('inferno')
    color = cmap(0.7)

    ax.plot(resolutions, rmse_scores, marker='o', color=color, linewidth=2, markersize=8, label='RMSE')
    ax.set_xscale('log')
    # ax.set_yscale('log')
    ax.set_xlabel('Range Resolution (m)', fontsize=16, fontname='Times New Roman')
    ax.set_ylabel('RMSE (g/cm³)', fontsize=16, fontname='Times New Roman')
    ax.set_ylim(0.07, 0.13)
    ax.grid(True, which="both", ls="--", linewidth=0.7, alpha=0.7)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()