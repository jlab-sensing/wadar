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

def monte_carlo_regression_feature_selection(feature_table, labels, data_dir, degree=1, n_iterations=100):
    """
    Test different feature sets using Monte Carlo simulation to determine the best performing features.

    Args:
        feature_table (pd.DataFrame):   DataFrame containing the features and labels.
        labels (np.ndarray):            Array of labels corresponding to the features.
        data_dir (str):                 Directory to save the feature table.
        n_iterations (int):             Number of iterations to run the Monte Carlo simulation.
        test_size (float):              Proportion of the dataset to include in the test split.

    Returns:
        np.ndarray:                     Array of features selected based on the best performing model.
        list:                           List of feature names selected based on the best performing model.
        np.ndarray:                     Array of labels corresponding to the selected features.
    """

    feature_array = feature_table.drop(columns=['Label']).values
    labels = feature_table['Label'].values

    best_features = []
    best_score = np.inf

    for i in range(n_iterations):
        top_n = np.random.randint(1, feature_array.shape[1] + 1)
        selected_indices = np.random.choice(feature_array.shape[1], top_n, replace=False)
        selected_features = feature_array[:, selected_indices]
        
        model, metrics = polynomial_regression(
            selected_features, labels, degree=degree, kfold_splits=5
        )

        if metrics['mae'] < best_score:
            print(f"Improved MAE from {best_score:.4f} to {metrics['mae']:.4f} at iteration {i+1}")

        if metrics['mae'] < best_score:
            best_score = metrics['mae']
            best_features = selected_indices

    feature_table_optimal = feature_table.copy().iloc[:, best_features]
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal, data_dir, f"models/feature_linear_regression_{degree}_monte_carlo.csv"
    )

    return feature_array[:, best_features], feature_table.columns[best_features].tolist(), labels