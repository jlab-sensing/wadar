import numpy as np
from scipy import signal

def novelda_digital_downconvert(raw_frame):
    """
    Function to apply a digital downcovert (DDC) to a high frequency radar
    signal. Brings signal to baseband frequencies and provides an analytic
    signal (i.e. I & Q, in-phase & quadrature, outputs). Inherited from
    NoveldaDDC.m.

    Args:
        raw_frame (np.ndarray):         Input radar frame data, shape (N,).

    Returns:
        baseband_signal (np.ndarray):   Baseband signal after DDC, shape (N,).
    """

    # These parameters are for true the X1-IPG1
    Fs = 39e9           # Mean system sampling rate for 4mm
    fL = 435e6;         # -10 dB low cutoff for pgen 0
    fH = 3165e6;        # -10 dB high cutoff
    Fc = (fH + fL) / 2

    # Digital Down-Convert parameters (normalized frequency index)
    N = len(raw_frame)                             
    freqIndex = Fc / Fs * N
    t = np.linspace(0, 1, N, endpoint=False)

    # Generate the complex sinusoid LO (local oscillator) to mix into the signal 
    phi = 0         # Phase offset?
    LO = np.sin(2 * np.pi * freqIndex * t + phi) + 1j * np.cos(2 * np.pi * freqIndex * t + phi)

    # Digital Downconvert (the DDC) via direct multiplication subtracting the mean removes DC offset
    rf_signal = raw_frame - np.mean(raw_frame)
    mixed = rf_signal * LO

    # LPF Design to eliminate the upper mixing frequencies after the DDC (21- tap hamming window)
    M = 20                                  # Filter order, equal to # of filter taps - 1. Inherited this from NoveldaDDC.m.
    window = np.hamming(M + 1)
    window /= np.sum(window[:(M//2 + 1)])   # Normalize the weights

    # Baseband signal using convolution (provides downcoverted, filtered analytic signal)
    baseband_signal = signal.convolve(mixed, window, mode='same')

    return baseband_signal

def remove_anomalies(X, threshold=50):
    """
    Replace values in X that deviate from the median by more than threshold with the median. This
    is something that has been done since the beginning of the project.

    Args:
        X (np.ndarray): Input array of shape (frames, range_bins, time_bins).
        threshold (float): Threshold for anomaly detection.
    Returns:
        np.ndarray: Array with anomalies replaced by the median.
    """
    
    X_clean = X.copy()
    for i in range(X_clean.shape[0]):
        for j in range(X_clean.shape[1]):
            med = np.median(X_clean[i, j, :])
            for k in range(X_clean.shape[2]):
                if np.abs(X_clean[i, j, k] - med) > threshold:
                    X_clean[i, j, k] = med
    return X_clean