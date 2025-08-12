import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.plotting_tools.data_plotting import plot_IQ_signals

from graphviz import Digraph
import matplotlib.pyplot as plt

def main():

    setup_logging(verbose=False)

    # Lab data visualization
    visualize_lab_data()

    # Flow chart of pipeline
    dspml_pipeline()

    visualize_previous_work_gpr()

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

if __name__ == "__main__":
    main()