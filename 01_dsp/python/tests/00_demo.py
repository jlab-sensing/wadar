import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader

# A bunch of figures to visually see the difference between soil compaction levels

if __name__ == "__main__":

    # load the dataset
    dataset_dir = "../data/dry-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    unique_Y, unique_Y_idx = np.unique(y, return_index=True)
    unique_Y = unique_Y[np.argsort(unique_Y)]

    X_abs = np.median(np.abs(X), axis=2)

    # # Plot each soil compaction level in a separate subplot

    # num_plots = len(unique_Y)
    # fig, axes = plt.subplots(num_plots, 1, figsize=(10, 2 * num_plots), sharex=True)

    # print(X.shape)
    # for i, label in enumerate(unique_Y):
    #     idx = np.where(y == label)[0]
    #     axes[i].plot(X[idx].T, alpha=0.5)
    #     axes[i].set_ylabel(f"Level {label}")
    #     axes[i].set_title(f"Soil Compaction Level: {label}")

    # axes[-1].set_xlabel("Feature Index")
    # plt.tight_layout()

    # Plot each soil compaction level in a single plot with different colors

    plt.figure(figsize=(10, 6))
    colors = plt.cm.viridis(np.linspace(0, 1, len(unique_Y)))
    for i, label in enumerate(unique_Y):
        idx = np.where(y == label)[0]
        # Only add label to the first line for each level
        plt.plot(X_abs[idx].T, alpha=0.3, color=colors[i], label=f"Level {label}" if len(idx) > 0 else None)
        # Remove label from all but the first line for each level
        if len(idx) > 1:
            for line in plt.gca().lines[-len(idx)+1:]:
                line.set_label(None)
    plt.xlabel("Feature Index")
    plt.ylabel("Median |Value|")
    plt.title("Soil Compaction Levels (All in One Plot)")
    plt.legend()
    plt.tight_layout()

    X_ang = np.median(np.angle(X), axis=2)

    plt.figure(figsize=(10, 6))
    colors = plt.cm.viridis(np.linspace(0, 1, len(unique_Y)))
    for i, label in enumerate(unique_Y):
        idx = np.where(y == label)[0]
        # Only add label to the first line for each level
        plt.plot(X_ang[idx].T, alpha=0.3, color=colors[i], label=f"Level {label}" if len(idx) > 0 else None)
        # Remove label from all but the first line for each level
        if len(idx) > 1:
            for line in plt.gca().lines[-len(idx)+1:]:
                line.set_label(None)
    plt.xlabel("Feature Index")
    plt.ylabel("Median |Value|")
    plt.title("Soil Compaction Levels (All in One Plot)")
    plt.legend()
    plt.tight_layout()
    plt.show()