"""Learned feature reduction using kernel principal component analysis."""

import logging
logger = logging.getLogger(__name__)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from sklearn.decomposition import KernelPCA
from sklearn.model_selection import ParameterGrid
from sklearn.metrics import mean_squared_error
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import Pipeline
from sklearn.model_selection import GridSearchCV
import warnings

from ...parameters import GRID_SEARCH_SCORING

class kPCALearnedFeatures:
    """
    Class for learned feature reduction using kernel principal component analysis (kPCA).

    This class separates the input complex-valued data into amplitude and angle components,
    applies kPCA to each, and can also apply kPCA to the combined features. The kernel is
    optimized to determine the best regression using a linear regression head.

    Attributes:
        n_components (int):         Number of principal components to retain.
        pca_amp (KernelPCA):        KernelPCA instance for amplitude features.
        pca_ang (KernelPCA):        KernelPCA instance for angle features.
        pca_com (KernelPCA):        KernelPCA instance for combined amplitude and angle features.
        X_amplitude (np.ndarray):   Amplitude part of the input data.
        X_phase (np.ndarray):       Unwrapped angle part of the input data.
        X_combined (np.ndarray):    Concatenated amplitude and angle features.
    """

    def __init__(self, X:np.ndarray, y:np.ndarray, n_components:int=None):
        """
        Initialize the kPCALearnedFeatures.

        Args:
            X (np.ndarray):     Raw radar data
            y (np.ndarray):     Labels
            n_components (int): Number of features desired
        """

        self.n_components = n_components
        self.y = y
        
        self.X_amplitude = self.preprocess(np.abs(X))
        self.X_phase = self.preprocess(np.unwrap(np.angle(X)))
        self.X_combined = self.preprocess(np.concatenate((np.abs(X), np.unwrap(np.angle(X))), axis=1))

        if (n_components % 2 == 1):
            logger.warning("Using odd number of components may yield odd results.")

    def full_monty(self):
        """
        Performs the entire dimensionality reduction process.

        Returns:
            reduced_amplitude (np.ndarray):     PCA-based features from amplitude
            reduced_phase (np.ndarray):         PCA-based features from phase
            reduced_combined (np.ndarray):      PCA-based features from amplitude and phase
        """

        amp_best_params, ang_best_params, com_best_params = self.tune()
        self.build_model(amp_best_params, ang_best_params, com_best_params)
        reduced_amplitude, reduced_phase, reduced_combined = self.dimensionality_reduction()
        return reduced_amplitude, reduced_phase, reduced_combined

    def preprocess(self, X:np.ndarray):
        """
        Preprocess radar data for KernelPCA class.

        Args:
            X (np.ndarray): Raw radar data

        Returns:
            X (np.ndarray): Preprocessed radar data
        """

        # Flatten input matrix
        N, R, T = X.shape
        X = X.reshape(N, R*T)

        return X
    
    def build_model(self, amp_best_params:dict, ang_best_params:dict, com_best_params:dict):
        """
        Builds the KernelPCA classes based on the provided parameters.

        Args:
            amp_best_params (dict): Best parameters for amplitude features.
            ang_best_params (dict): Best parameters for angle features.
            com_best_params (dict): Best parameters for combined features.
        """

        # Get the parameters with their corresponding names
        amp_params = {k.replace("kpca__", ""): v for k, v in amp_best_params.items()}
        ang_params = {k.replace("kpca__", ""): v for k, v in ang_best_params.items()}
        com_params = {k.replace("kpca__", ""): v for k, v in com_best_params.items()}

        # Build model with optimal parameters
        self.pca_amp = KernelPCA(n_components=self.n_components, **amp_params)
        self.pca_ang = KernelPCA(n_components=self.n_components, **ang_params)
        self.pca_com = KernelPCA(n_components=self.n_components, **com_params)
    
    def dimensionality_reduction(self):
        """
        Perform KernelPCA on the input data and returns the reduced data.

        Returns:
            reduced_amp (np.ndarray):       KernelPCA-based features from amplitude
            reduced_ang (np.ndarray):       KernelPCA-based features from phase
            reduced_combined (np.ndarray):  KernelPCA-based features from amplitude and phase
        """

        logger.info("Performing PCA dimensionality reduction.")

        # Fit all models to corresponding features
        reduced_amp = self.pca_amp.fit_transform(self.X_amplitude)  
        reduced_ang = self.pca_ang.fit_transform(self.X_phase)
        reduced_combined = self.pca_com.fit_transform(self.X_combined)

        return reduced_amp, reduced_ang, reduced_combined
    
    def tune(self):
        """
        Finds optimal KernelPCA parameters using a regression head.

        Returns:
            amp_best_params (dict): Best parameters for amplitude features.
            ang_best_params (dict): Best parameters for angle features.
            com_best_params (dict): Best parameters for combined features.
        """

        # Using a pipeline with linear regression to optimize for linear separability
        pipe = Pipeline([
            ('kpca', KernelPCA(fit_inverse_transform=True)),
            ('clf', LinearRegression())
        ])

        param_grid = {
            'kpca__kernel': ['linear', 'rbf', 'poly', 'sigmoid'],
            'kpca__gamma': [0.001, 0.01, 0.1, 1],
            'kpca__degree': [2, 3, 4]  # Only for poly
        }

        grid_search = GridSearchCV(pipe, param_grid, cv=5, scoring=GRID_SEARCH_SCORING) 

        with warnings.catch_warnings():
            warnings.simplefilter("ignore") # Suppresses warnings that rise from poor parameters in the parameter grid search

            logger.info("Tuning with amplitude for kPCA")
            grid_search.fit(self.X_amplitude, self.y)
            amp_best_params = grid_search.best_params_
            logger.info(f"Best params for amplitude: {amp_best_params}")

            logger.info("Tuning with angle for kPCA")
            grid_search.fit(self.X_phase, self.y)
            ang_best_params = grid_search.best_params_
            logger.info(f"Best params for angle: {ang_best_params}")

            logger.info("Tuning with combined for kPCA")
            grid_search.fit(self.X_combined, self.y)
            com_best_params = grid_search.best_params_
            logger.info(f"Best params for combined: {com_best_params}")

        return amp_best_params, ang_best_params, com_best_params

    
    def transform(self, X:np.ndarray):
        """
        Apply the previously fitted KernelPCA to new data.

        Args:
            X (np.ndarray): New input data of shape (samples, features).

        Returns:
            transformed_amplitude (np.ndarray):     KernelPCA-based features from amplitude
            transformed_phase (np.ndarray):         KernelPCA-based features from phase
            transformed_combined (np.ndarray):      KernelPCA-based features from amplitude and phase
        """

        X_amplitude = np.abs(X)
        X_phase = np.unwrap(np.angle(X))
        X_combined = np.concatenate((X_amplitude, X_phase), axis=1)

        X_amplitude_flat = self.preprocess(X_amplitude)
        X_phase_flat = self.preprocess(X_phase)
        X_combined_flat = self.preprocess(X_combined)

        transformed_amplitude = self.pca_amp.transform(X_amplitude_flat)
        transformed_phase = self.pca_ang.transform(X_phase_flat)
        transformed_combined = self.pca_com.transform(X_combined_flat)

        return transformed_amplitude, transformed_phase, transformed_combined
