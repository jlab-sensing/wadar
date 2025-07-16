import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import ConfusionMatrixDisplay, confusion_matrix
import seaborn as sns

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

def plot_phase_median_unique(X, y):
    unique_Y, unique_Y_idx = np.unique(y, return_index=True)
    X = np.angle(X)
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

def plot_confusion_matrix(y_labels, y_pred):
    cm = confusion_matrix(y_labels, y_pred)
    labels = np.unique(y_labels)

    plt.figure(figsize=(8, 6))
    sns.set(font_scale=1.4)
    ax = sns.heatmap(
        cm,
        annot=True,
        fmt='d',
        cmap='Blues',
        xticklabels=labels,
        yticklabels=labels,
        cbar=False,
        linewidths=0.5,
        linecolor='gray',
        square=True,
        annot_kws={"size": 16, "weight": "bold"}
    )
    ax.set_xlabel('Predicted Label', fontsize=16, weight='bold')
    ax.set_ylabel('True Label', fontsize=16, weight='bold')
    plt.xticks(fontsize=14, weight='bold', rotation=45)
    plt.yticks(fontsize=14, weight='bold', rotation=0)
    plt.tight_layout()
    plt.show()