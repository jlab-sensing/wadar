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
    plt.figure()
    # Plot Median Frames for each unique label
    for i, j in enumerate(unique_Y_idx):
        plt.plot(np.median(X[j, :, :], axis=1), label=f"Label: {unique_Y[i]:.2f}")
    plt.tight_layout()
    plt.title("Median Frames for Unique Labels")
    plt.xlabel("Range Bins")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.show()