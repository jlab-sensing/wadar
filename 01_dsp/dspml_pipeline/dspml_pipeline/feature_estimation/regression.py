import logging
logger = logging.getLogger(__name__)

import time
import numpy as np
import sys
import warnings
from sklearn.model_selection import KFold
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.linear_model import Ridge
from sklearn.pipeline import Pipeline
from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.model_selection import GridSearchCV

from ..parameters import KFOLD_SPLITS, RANDOM_SEED, num2label

class RidgeRegression:
    def __init__(self):
        self.model = None
        self.metrics = None

    def full_monty(self, feature_array, labels):
        metrics = self.cross_validate(feature_array=feature_array, labels=labels)
        alpha, degree = self.tune(feature_array, labels)
        logger.info(f"Final model trained: Ridge Regression with alpha={alpha}, degree={degree}")
        model = self.train(feature_array, labels, alpha, degree)
        return model, metrics

    def build_model(self, alpha, degree):
        return Pipeline([
            ('poly', PolynomialFeatures(degree=degree, include_bias=False)),
            ('scaler', StandardScaler()),
            ('ridge', Ridge(alpha=alpha, random_state=RANDOM_SEED))
        ])

    def cross_validate(self, feature_array, labels):
        X, y = feature_array, labels
        kf = KFold(n_splits=KFOLD_SPLITS, shuffle=True, random_state=RANDOM_SEED)

        # Store metrics for each fold
        metrics = {'mae': [], 'rmse': [], 'accuracy': [], 'inference_time': [], 'training_time': []}

        # Cross-validation loop
        fold_num = 0
        for train_index, test_index in kf.split(X):
            
            fold_num += 1
            X_train, X_test = X[train_index], X[test_index]
            y_train, y_test = y[train_index], y[test_index]

            alpha, degree = self.tune(X_train, y_train)

            # Time training
            train_start = time.time()
            model = self.train(X_train, y_train, alpha, degree)
            training_time = time.time() - train_start

            # Time inference
            inference_start = time.time()
            y_pred = model.predict(X_test)
            inference_time = time.time() - inference_start

            # Calculate metrics
            mae = mean_absolute_error(y_test, y_pred)
            rmse = np.sqrt(mean_squared_error(y_test, y_pred))
            
            # Calculate accuracy using num2label
            y_test_labels = [num2label(label) for label in y_test]
            y_pred_labels = [num2label(pred) for pred in y_pred]
            accuracy = np.mean([pred == true for pred, true in zip(y_pred_labels, y_test_labels)])

            # Store metrics
            metrics['mae'].append(mae)
            metrics['rmse'].append(rmse)
            metrics['accuracy'].append(accuracy)
            metrics['inference_time'].append(inference_time)
            metrics['training_time'].append(training_time)

            logger.info(f"Fold {fold_num}/{KFOLD_SPLITS} - MAE: {mae:.2f}, RMSE: {rmse:.2f}, "
                        f"Accuracy: {100*accuracy:.2f}%, Training time: {1000*training_time:.2f}ms, "
                        f"Inference time: {1000*inference_time:.2f}ms")

        # Store averaged metrics
        metrics = {
            "mae": np.mean(metrics['mae']),
            "rmse": np.mean(metrics['rmse']),
            "accuracy": np.mean(metrics['accuracy']),
            "inference_time": np.mean(metrics['inference_time']),
            "training_time": np.mean(metrics['training_time'])
        }

        logger.info(f"Average metrics - MAE: {metrics['mae']:.2f}, RMSE: {metrics['rmse']:.2f}, "
                   f"Accuracy: {100*metrics['accuracy']:.2f}%, Training time: {1000*metrics['training_time']:.2f}ms, "
                   f"Inference time: {1000*metrics['inference_time']:.2f}ms")

        return metrics
    
    def tune(self, X, y):

        with warnings.catch_warnings():
            warnings.simplefilter("ignore") # Suppresses to warnings that rise from poor parameters in the parameter grid search

            pipe = Pipeline([
                ('poly', PolynomialFeatures(include_bias=False)),
                ('scaler', StandardScaler()),
                ('ridge', Ridge(random_state=RANDOM_SEED))
            ])

            alpha_values = [0.1, 1, 10, 100, 1000]
            degrees = [1, 2, 3, 4]
            

            param_grid = {
                'ridge__alpha': alpha_values,
                'poly__degree': degrees,
            }

            grid_search = GridSearchCV(pipe, param_grid, cv=KFOLD_SPLITS, scoring='neg_mean_absolute_error')
            grid_search.fit(X, y)

            best_params = grid_search.best_params_
            alpha = best_params['ridge__alpha']
            degree = best_params['poly__degree']

        return alpha, degree
    
    def train(self, X, y, alpha, degree):

        self.model = self.build_model(alpha=alpha, degree=degree)
        self.model.fit(X, y)

        return self.model

    def estimate(self, X):
        if self.model is None:
            logger.error("Model has not been fitted yet. Call train() first.")
            sys.exit(1)
        return self.model.predict(X)