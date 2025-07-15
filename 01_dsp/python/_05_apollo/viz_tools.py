import numpy as np
import matplotlib.pyplot as plt

def plot_median_frame(X, frame_index=0):
    X = np.abs(X)
    plt.plot(np.median(X[frame_index, :, :], axis=1))
    plt.title(f"Median Frame {frame_index}")
    plt.xlabel("Range Bins")
    plt.ylabel("Amplitude")
    plt.grid()
    plt.tight_layout()
    plt.show()

def plot_median_unique(X, y):
    unique_Y, unique_Y_idx = np.unique(y, return_index=True)
    X = np.abs(X)
    plt.figure(figsize=(7, 4))
    cmap = plt.get_cmap('viridis')
    colors = cmap(np.linspace(0, 1, len(unique_Y)))
    for i, j in enumerate(unique_Y_idx):
        plt.plot(
            np.median(X[j, :, :], axis=1),
            label=f"Label: {unique_Y[i]:.2f}",
            color=colors[i],
            linewidth=2
        )
    plt.title("Median Frames for Unique Labels", fontsize=14, fontweight='bold')
    plt.xlabel("Range Bins", fontsize=12)
    plt.ylabel("Amplitude", fontsize=12)
    plt.legend(frameon=False, fontsize=10, loc='best')
    plt.grid(True, linestyle='--', linewidth=0.5, alpha=0.7)
    plt.tight_layout(pad=1.5)
    plt.tick_params(axis='both', which='major', labelsize=10)
    plt.show()