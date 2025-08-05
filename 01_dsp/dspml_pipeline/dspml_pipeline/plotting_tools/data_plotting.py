import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.set_loglevel("warning") 

def plot_IQ_signals(X, y):
    """
    Plot the mean and standard deviation of the absolute value and angle of the I/Q signals using a colorbar.
    Generated almost entirely by AI.

    Args:
        X (np.ndarray):         Input data of shape (samples, range_bins, time_bins).
        y (np.ndarray):         Labels of shape (samples,).
    """

    unique_Y = np.unique(y)

    # Plotting amplitude

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
        plt.plot(mean_curve, color=color, linewidth=2)
        plt.fill_between(np.arange(mean_curve.size), mean_curve - std_curve, mean_curve + std_curve,
                         color=color, alpha=0.2)
    
    plt.xlabel("Range Bin", fontsize=14)
    plt.ylabel("Amplitude of I/Q Signal", fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label("Compaction Level", fontsize=13)
    
    plt.tight_layout()

    # Plotting angle

    X_angle = np.median(np.angle(X), axis=2)
    X_angle = np.unwrap(X_angle)

    fig, ax = plt.subplots(figsize=(12, 7))

    sorted_unique_Y = np.sort(unique_Y)
    norm = mpl.colors.Normalize(vmin=sorted_unique_Y.min(), vmax=sorted_unique_Y.max())
    cmap = plt.cm.inferno

    for label in sorted_unique_Y:
        idx = np.where(y == label)[0]
        color = cmap(norm(label))

        mean_curve = np.mean(X_angle[idx], axis=0)
        std_curve = np.std(X_angle[idx], axis=0)
        plt.plot(mean_curve, color=color, linewidth=2)
        plt.fill_between(np.arange(mean_curve.size), mean_curve - std_curve, mean_curve + std_curve,
                         color=color, alpha=0.2)
    
    plt.xlabel("Range Bin", fontsize=14)
    plt.ylabel("Amplitude of I/Q Signal", fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label("Compaction Level", fontsize=13)
    
    plt.tight_layout()

    plt.show()
