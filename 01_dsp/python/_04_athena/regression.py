import numpy as np
from sklearn.preprocessing import StandardScaler  # Fixed import
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split, KFold
from sklearn.metrics import mean_absolute_error, r2_score, mean_squared_error  # Added MSE
import pandas as pd
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import RidgeCV, Ridge
from sklearn.pipeline import Pipeline
from _03_hephaestus import feature_tools
import time
from _06_hermes.parameters import num2label, RANDOM_SEED, KFOLD_SPLITS

from sklearn.model_selection import KFold

def polynomial_regression(feature_array, labels, degree=1, kfold_splits=KFOLD_SPLITS, alpha=1000, **kwargs):

    X, y = feature_array, labels
    kf = KFold(n_splits=kfold_splits, shuffle=True, random_state=RANDOM_SEED)

    # Store metrics for each fold
    metrics = {'mae': [], 'rmse': [], 'r2': [], 'accuracy': [], 'inference_time': [], 'training_time': []}

    # Cross-validation loop
    for train_index, test_index in kf.split(X):
        X_train, X_test = X[train_index], X[test_index]
        y_train, y_test = y[train_index], y[test_index]

        # Create and train model
        model = Pipeline([
            ('poly', PolynomialFeatures(degree=degree, include_bias=False)),
            ('scaler', StandardScaler()),
            ('ridge', Ridge(alpha=alpha, random_state=RANDOM_SEED))
        ])

        # Time training
        train_start = time.time()
        model.fit(X_train, y_train)
        training_time = time.time() - train_start

        # Time inference
        inference_start = time.time()
        y_pred = model.predict(X_test)
        inference_time = time.time() - inference_start

        # Calculate metrics
        mae = mean_absolute_error(y_test, y_pred)
        rmse = np.sqrt(mean_squared_error(y_test, y_pred))
        r2 = r2_score(y_test, y_pred)
        
        # Calculate accuracy using num2label
        y_test_labels = [num2label(label) for label in y_test]
        y_pred_labels = [num2label(pred) for pred in y_pred]
        accuracy = np.mean([pred == true for pred, true in zip(y_pred_labels, y_test_labels)])

        # Store metrics
        metrics['mae'].append(mae)
        metrics['rmse'].append(rmse)
        metrics['r2'].append(r2)
        metrics['accuracy'].append(accuracy)
        metrics['inference_time'].append(inference_time)
        metrics['training_time'].append(training_time)

    # Train final model on full dataset
    final_model = Pipeline([
        ('poly', PolynomialFeatures(degree=degree, include_bias=False)),
        ('scaler', StandardScaler()),
        ('ridge', Ridge(alpha=alpha, random_state=RANDOM_SEED))
    ])
    final_model.fit(X, y)

    # Return model and averaged metrics
    return final_model, {
        "mae": np.mean(metrics['mae']),
        "rmse": np.mean(metrics['rmse']),
        "r2": np.mean(metrics['r2']),
        "accuracy": np.mean(metrics['accuracy']),
        "inference_time": np.mean(metrics['inference_time']),
        "training_time": np.mean(metrics['training_time'])
    }