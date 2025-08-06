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
    PCAProcessor is a class to perform Principal Component Analysis (PCA) on radar data.

    Parameters:
        X (np.ndarray):         Input data of shape (samples, features).
        n_components (int):     Number of components to keep.
    """

    def __init__(self, X, y, n_components=None):
        """
        Initialize the PCAProcessor.
        """

        self.n_components = n_components
        self.y = y
        
        self.X_amp = self.preprocess(np.abs(X))
        self.X_ang = self.preprocess(np.unwrap(np.angle(X)))
        self.X_com = self.preprocess(np.concatenate((np.abs(X), np.unwrap(np.angle(X))), axis=1))

        if (n_components % 2 == 1):
            logger.warning("Using odd number of components may yield odd results.")

    def full_monty(self):
        amp_best_params, ang_best_params, com_best_params = self.tune()
        self.build_model(amp_best_params, ang_best_params, com_best_params)
        reduced_amp, reduced_ang, reduced_combined = self.dimensionality_reduction()
        return reduced_amp, reduced_ang, reduced_combined

    def preprocess(self, X):

        # Flatten input matrix
        N, R, T = X.shape
        X = X.reshape(N, R*T)

        return X
    
    def build_model(self, amp_best_params, ang_best_params, com_best_params):

        # Get the parameters with their corresponding names
        amp_params = {k.replace("kpca__", ""): v for k, v in amp_best_params.items()}
        ang_params = {k.replace("kpca__", ""): v for k, v in ang_best_params.items()}
        com_params = {k.replace("kpca__", ""): v for k, v in com_best_params.items()}

        # Build model with optimal parameters
        self.pca_amp = KernelPCA(n_components=self.n_components, **amp_params)
        self.pca_ang = KernelPCA(n_components=self.n_components, **ang_params)
        self.pca_com = KernelPCA(n_components=self.n_components, **com_params)
        reduced_amp, reduced_ang, reduced_combined = self.dimensionality_reduction()

        return reduced_amp, reduced_ang, reduced_combined
    
    def dimensionality_reduction(self):
        """
        Perform PCA on the input data and return the reduced data.

        Returns:
            np.ndarray: Reduced data of shape (samples, n_components).
        """

        logger.info("Performing PCA dimensionality reduction.")

        # Fit all models to corresponding features
        reduced_amp = self.pca_amp.fit_transform(self.X_amp)  
        reduced_ang = self.pca_ang.fit_transform(self.X_ang)
        reduced_combined = self.pca_com.fit_transform(self.X_com)

        return reduced_amp, reduced_ang, reduced_combined
    
    def tune(self):

        # Using a pipeline with lienar regression to optimize for linear separability
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
            grid_search.fit(self.X_amp, self.y)
            amp_best_params = grid_search.best_params_
            logger.info(f"Best params for amplitude: {amp_best_params}")

            logger.info("Tuning with angle for kPCA")
            grid_search.fit(self.X_ang, self.y)
            ang_best_params = grid_search.best_params_
            logger.info(f"Best params for angle: {ang_best_params}")

            logger.info("Tuning with combined for kPCA")
            grid_search.fit(self.X_com, self.y)
            com_best_params = grid_search.best_params_
            logger.info(f"Best params for combined: {com_best_params}")

        return amp_best_params, ang_best_params, com_best_params

    
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
