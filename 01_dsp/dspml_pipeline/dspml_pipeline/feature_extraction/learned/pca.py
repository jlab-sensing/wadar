"""Learned feature reduction using principal component analysis."""

import logging
logger = logging.getLogger(__name__)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from sklearn.decomposition import PCA

class PCALearnedFeatures:
    """
    Class for learned feature reduction using principal component analysis (PCA).

    This class separates the input complex-valued data into amplitude and angle components,
    applies PCA to each, and can also apply PCA to the combined features.

    Attributes:
        n_components (int):         Number of principal components to retain.
        pca_amp (PCA):              PCA instance for amplitude features.
        pca_ang (PCA):              PCA instance for angle features.
        pca_com (PCA):              PCA instance for combined amplitude and angle features.
        X_amplitude (np.ndarray):   Amplitude part of the input data.
        X_phase (np.ndarray):       Unwrapped angle part of the input data.
        X_combined (np.ndarray):    Concatenated amplitude and angle features.
    """

    def __init__(self, X: np.ndarray, n_components:int = None):
        """
        Initialize the PCALearnedFeatures.

        Args:
            X (np.ndarray):     Raw radar data
            n_components (int): Number of features desired
        """

        self.n_components = n_components

        self.pca_amplitude = PCA(n_components=self.n_components)
        self.pca_phase = PCA(n_components=self.n_components)
        self.pca_combined = PCA(n_components=self.n_components)
        
        self.X_amplitude = np.abs(X)
        self.X_phase = np.unwrap(np.angle(X))
        self.X_combined = np.concatenate((self.X_amplitude, self.X_phase), axis=1)

        if (n_components % 2 == 1):
            logger.warning("Using odd number of components may yield odd results.")

    def full_monty(self):
        """
        Performs the entire dimensionality reduction process.

        Returns:
            reduced_amplitude (np.ndarray):       PCA-based features from amplitude
            reduced_phase (np.ndarray):         PCA-based features from phase
            reduced_combined (np.ndarray):      PCA-based features from amplitude and phase
        """

        reduced_amplitude, reduced_phase, reduced_combined = self.dimensionality_reduction()
        return reduced_amplitude, reduced_phase, reduced_combined

    def preprocess(self, X:np.ndarray):
        """
        Preprocess radar data for PCA class.

        Args:
            X (np.ndarray): Raw radar data

        Returns:
            X (np.ndarray): Preprocessed radar data
        """

        # Flatten input matrix
        N, R, T = X.shape
        X = X.reshape(N, R*T)
        return X
    
    def dimensionality_reduction(self):
        """
        Perform PCA on the input data and returns the reduced data.

        Returns:
            reduced_amp (np.ndarray):       PCA-based features from amplitude
            reduced_ang (np.ndarray):       PCA-based features from phase
            reduced_combined (np.ndarray):  PCA-based features from amplitude and phase
        """

        logger.info("Performing PCA dimensionality reduction.")

        X_amplitude = self.preprocess(self.X_amplitude)
        reduced_amp = self.pca_amplitude.fit_transform(X_amplitude)  

        X_phase = self.preprocess(self.X_phase)
        reduced_ang = self.pca_phase.fit_transform(X_phase)

        X_combined = self.preprocess(self.X_combined)
        reduced_combined = self.pca_combined.fit_transform(X_combined)

        return reduced_amp, reduced_ang, reduced_combined
    
    def transform(self, X:np.ndarray):
        """
        Apply the previously fitted PCA to new data.

        Args:
            X (np.ndarray): New input data of shape (samples, features).

        Returns:
            transformed_amplitude (np.ndarray):     PCA-based features from amplitude
            transformed_phase (np.ndarray):         PCA-based features from phase
            transformed_combined (np.ndarray):      PCA-based features from amplitude and phase
        """

        X_amplitude = np.abs(X)
        X_phase = np.unwrap(np.angle(X))
        X_combined = np.concatenate((X_amplitude, X_phase), axis=1)

        X_amplitude_flat = self.preprocess(X_amplitude)
        X_phase_flat = self.preprocess(X_phase)
        X_combined_flat = self.preprocess(X_combined)

        transformed_amplitude = self.pca_amplitude.transform(X_amplitude_flat)
        transformed_phase = self.pca_phase.transform(X_phase_flat)
        transformed_combined = self.pca_combined.transform(X_combined_flat)

        return transformed_amplitude, transformed_phase, transformed_combined
