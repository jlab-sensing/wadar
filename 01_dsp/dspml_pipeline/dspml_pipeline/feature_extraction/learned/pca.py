import logging
logger = logging.getLogger(__name__)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from sklearn.decomposition import PCA

class PCALearnedFeatures:
    """
    PCAProcessor is a class to perform Principal Component Analysis (PCA) on radar data.

    Parameters:
        X (np.ndarray):         Input data of shape (samples, features).
        n_components (int):     Number of components to keep.
    """

    def __init__(self, X, n_components=None):
        """
        Initialize the PCAProcessor.
        """

        self.n_components = n_components

        self.pca_amp = PCA(n_components=self.n_components)
        self.pca_ang = PCA(n_components=self.n_components)
        self.pca_com = PCA(n_components=self.n_components)
        
        self.X_amp = np.abs(X)
        self.X_ang = np.unwrap(np.angle(X))
        self.X_com = np.concatenate((self.X_amp, self.X_ang), axis=1)

        if (n_components % 2 == 1):
            logger.warning("Using odd number of components may yield odd results.")

    def full_monty(self):
        reduced_amp, reduced_ang, reduced_combined = self.dimensionality_reduction()
        return reduced_amp, reduced_ang, reduced_combined

    def preprocess(self, X):

        # Flatten input matrix
        N, R, T = X.shape
        X = X.reshape(N, R*T)
        return X
    
    def dimensionality_reduction(self):
        """
        Perform PCA on the input data and return the reduced data.

        Returns:
            np.ndarray: Reduced data of shape (samples, n_components).
        """

        logger.info("Performing PCA dimensionality reduction.")

        X_amp = self.preprocess(self.X_amp)
        reduced_amp = self.pca_amp.fit_transform(X_amp)  

        X_ang = self.preprocess(self.X_ang)
        reduced_ang = self.pca_ang.fit_transform(X_ang)

        X_com = self.preprocess(self.X_com)
        reduced_combined = self.pca_com.fit_transform(X_com)

        return reduced_amp, reduced_ang, reduced_combined
    
    def transform(self, X):
        """
        Apply the previously fitted PCA to new data.

        Parameters:
            X (np.ndarray): New input data of shape (samples, features).

        Returns:
            tuple: Transformed amplitude, angle, and combined data.
        """

        X_amp = np.abs(X)
        X_ang = np.unwrap(np.angle(X))
        X_com = np.concatenate((X_amp, X_ang), axis=1)

        X_amp_flat = self.preprocess(X_amp)
        X_ang_flat = self.preprocess(X_ang)
        X_com_flat = self.preprocess(X_com)

        transformed_amp = self.pca_amp.transform(X_amp_flat)
        transformed_ang = self.pca_ang.transform(X_ang_flat)
        transformed_combined = self.pca_com.transform(X_com_flat)

        return transformed_amp, transformed_ang, transformed_combined
