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

def plot_IQ_signals(X, y):
    unique_Y, unique_Y_idx = np.unique(y, return_index=True)
    unique_Y = unique_Y[np.argsort(unique_Y)]

    X_abs = np.median(np.abs(X), axis=2)

    plt.figure(figsize=(12, 7))

    # color map for increasing soil compaction levels
    sorted_indices = np.argsort(unique_Y)
    sorted_unique_Y = unique_Y[sorted_indices]
    colors = plt.cm.viridis(np.linspace(0, 1, len(sorted_unique_Y)))

    for i, label in enumerate(sorted_unique_Y):
        idx = np.where(y == label)[0]

        # plotting the mean and median as a line with a shaded area
        # so the plot looks less cluttered
        mean_curve = np.mean(X_abs[idx], axis=0)
        std_curve = np.std(X_abs[idx], axis=0)
        plt.plot(mean_curve, color=colors[i], label=f"Level {label:.2f}", linewidth=2)
        plt.fill_between(np.arange(mean_curve.size), mean_curve - std_curve, mean_curve + std_curve,
                         color=colors[i], alpha=0.2)
    
    plt.xlabel("Range Bin", fontsize=14)
    plt.ylabel("Absolute Value of I/Q Signal", fontsize=14)
    plt.legend(title="Compaction Level", fontsize=12, title_fontsize=13, loc="best")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()

    X_angle = np.median(np.angle(X), axis=2)

    plt.figure(figsize=(12, 7))

    for i, label in enumerate(sorted_unique_Y):
        idx = np.where(y == label)[0]

        # plotting the mean and median as a line with a shaded area
        # so the plot looks less cluttered
        mean_curve = np.mean(X_angle[idx], axis=0)
        std_curve = np.std(X_angle[idx], axis=0)
        plt.plot(mean_curve, color=colors[i], label=f"Level {label:.2f}", linewidth=2)
        plt.fill_between(np.arange(mean_curve.size), mean_curve - std_curve, mean_curve + std_curve,
                         color=colors[i], alpha=0.2)

    plt.xlabel("Range Bin", fontsize=14)
    plt.ylabel("Angle of I/Q Signal", fontsize=14)
    plt.legend(title="Compaction Level", fontsize=12, title_fontsize=13, loc="best")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()

def plot_regression(y_true, y_pred):
    plt.figure(figsize=(8, 6))
    plt.scatter(y_true, y_pred, alpha=0.7, color='royalblue', edgecolor='k', s=60)
    plt.plot([min(y_true), max(y_true)], [min(y_true), max(y_true)],
                color='darkred', linestyle='--', linewidth=2, label='Ideal Fit')
    # plt.title('True vs Predicted Values', fontsize=16)
    plt.xlabel('True Values', fontsize=14)
    plt.ylabel('Predicted Values', fontsize=14)
    plt.legend(fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()