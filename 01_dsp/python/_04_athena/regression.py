import numpy as np
from sklearn.discriminant_analysis import StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, r2_score
import pandas as pd
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import RidgeCV, Ridge
from sklearn.pipeline import Pipeline
from _03_hephaestus import feature_tools
import time
from _06_hermes.parameters import num2label, RANDOM_SEED, KFOLD_SPLITS

from sklearn.model_selection import KFold

def polynomial_regression(feature_array, labels, degree=1, kfold_splits=KFOLD_SPLITS, **kwargs):
    """
    Performs regularized ridge polynomial regression on the provided features and labels.

    Args:
        feature_array (np.ndarray): Array of features of shape (samples, features).
        labels (np.ndarray):        Array of labels of shape (samples,).
        degree (int):               Degree of the polynomial features.
        kfold_splits (int):         Number of folds for cross-validation.

    Returns:
        None:                       (Model is fit per fold; averaging results.)
        dict:                       Dictionary with averaged metrics: MAE, RMSE, R2, Accuracy, Inference Time, Training Time.
    """

    X = feature_array
    y = labels

    kf = KFold(n_splits=kfold_splits, shuffle=True, random_state=RANDOM_SEED)

    mae_list = []
    rmse_list = []
    r2_list = []
    accuracy_list = []
    inference_time_list = []
    training_time_list = []

    for train_index, test_index in kf.split(X):
        X_train, X_test = X[train_index], X[test_index]
        y_train, y_test = y[train_index], y[test_index]

        model = Pipeline([
            ('poly', PolynomialFeatures(degree=degree, include_bias=False)),
            ('scaler', StandardScaler()),
            ('ridge', Ridge(alpha=1000))
        ])

        train_start = time.time()
        model.fit(X_train, y_train)
        training_time = time.time() - train_start

        time_start = time.time()
        y_pred = model.predict(X_test)
        inference_time = time.time() - time_start

        mae_list.append(mean_absolute_error(y_test, y_pred))
        rmse_list.append(np.sqrt(np.mean((y_test - y_pred) ** 2)))
        r2_list.append(r2_score(y_test, y_pred))

        y_labels = [num2label(label) for label in y_test]
        fold_accuracy = np.mean([num2label(pred) == y_true for pred, y_true in zip(y_pred, y_labels)])
        accuracy_list.append(fold_accuracy)
        inference_time_list.append(inference_time)
        training_time_list.append(training_time)

    model = Pipeline([
        ('poly', PolynomialFeatures(degree=degree, include_bias=False)),
        ('scaler', StandardScaler()),
        ('ridge', Ridge(alpha=1000))
    ])

    model.fit(X, y)

    return model, {
        "mae": np.mean(mae_list),
        "rmse": np.mean(rmse_list),
        "r2": np.mean(r2_list),
        "accuracy": np.mean(accuracy_list),
        "inference_time": np.mean(inference_time_list),
        "training_time": np.mean(training_time_list)
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