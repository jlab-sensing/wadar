import logging
logger = logging.getLogger(__name__)

import time
import numpy as np
import sys
import warnings
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestRegressor
from sklearn.pipeline import Pipeline
from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.model_selection import GridSearchCV

from ..parameters import KFOLD_SPLITS, RANDOM_SEED, num2label, GRID_SEARCH_SCORING

class RandomForest:
    def __init__(self, tune_model_params:bool = True):
        self.model = None
        self.metrics = None
        self.tune_model_params = tune_model_params

    def full_monty(self, feature_array, labels):
        metrics = self.cross_validate(feature_array=feature_array, labels=labels)
        n_estimators, max_depth = self.tune(feature_array, labels)
        logger.info(f"Final model trained: Random Forest with n_estimators={n_estimators}, max_depth={max_depth}")
        model = self.train(feature_array, labels, n_estimators, max_depth)
        return model, metrics

    def build_model(self, n_estimators, max_depth):
        return Pipeline([
            ('scaler', StandardScaler()),
            ('rf', RandomForestRegressor(n_estimators=n_estimators, max_depth=max_depth, random_state=RANDOM_SEED))
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

            n_estimators, max_depth = self.tune(X_train, y_train)

            # Time training
            train_start = time.time()
            model = self.train(X_train, y_train, n_estimators, max_depth)
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

        if not self.tune_model_params:
            n_estimators = 100
            max_depth = None
            return n_estimators, max_depth

        with warnings.catch_warnings():
            warnings.simplefilter("ignore") # Suppresses warnings that rise from poor parameters in the parameter grid search

            pipe = Pipeline([
                ('scaler', StandardScaler()),
                ('rf', RandomForestRegressor(random_state=RANDOM_SEED))
            ])

            n_estimators_values = [50, 100, 200, 300]
            max_depth_values = [None, 5, 10, 15, 20]
            

            param_grid = {
                'rf__n_estimators': n_estimators_values,
                'rf__max_depth': max_depth_values,
            }

            grid_search = GridSearchCV(pipe, param_grid, cv=KFOLD_SPLITS, scoring=GRID_SEARCH_SCORING)
            grid_search.fit(X, y)

            best_params = grid_search.best_params_
            n_estimators = best_params['rf__n_estimators']
            max_depth = best_params['rf__max_depth']

        return n_estimators, max_depth
    
    def train(self, X, y, n_estimators, max_depth):

        self.model = self.build_model(n_estimators=n_estimators, max_depth=max_depth)
        self.model.fit(X, y)

        return self.model

    def estimate(self, X):
        if self.model is None:
            logger.error("Model has not been fitted yet. Call train() first.")
            sys.exit(1)
        return self.model.predict(X)
