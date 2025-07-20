import json
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _06_hermes.bulk_density_labels import bulk_density_to_label
from sklearn.linear_model import SGDClassifier, SGDRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, r2_score
import pandas as pd
import numpy as np
from sklearn.model_selection import GridSearchCV
from _03_hephaestus import feature_tools
import time
from _06_hermes.parameters import num2label, RANDOM_SEED

def sgd_regression(feature_array, labels, test_size, eta0=0.001, max_iter=1000000, tol=1e-10):
    """
    Performs SGD regression on the given feature array and labels.

    Args:
        feature_array (np.ndarray):    Array of features of shape (samples, features).
        labels (np.ndarray):           Array of labels of shape (samples,).
        test_size (float):             Proportion of the dataset to include in the test split.
        eta0 (float):                  Initial learning rate.
        max_iter (int):                Maximum number of iterations for the SGD regressor.
        tol (float):                   Tolerance for stopping criteria.

    Returns:
        est (SGDRegressor):            Trained SGD regressor model.
        metrics (dict):                Dictionary containing evaluation metrics such as MAE, R2, accuracy, and inference time.
    """

    X_train, X_test, y_train, y_test = train_test_split(feature_array, labels, test_size=test_size, random_state=RANDOM_SEED)

    est = make_pipeline(
        StandardScaler(),
        SGDRegressor(
            eta0=eta0,
            max_iter=max_iter,
            tol=tol
        )
    )

    est.fit(X_train, y_train)

    time_start = time.time()
    y_pred = est.predict(X_test)
    inference_time = time.time() - time_start

    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    y_labels = [num2label(label) for label in y_test]
    accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

    metrics = {'mae': mae, 'r2': r2, 'inference_time': inference_time, 'accuracy': accuracy}

    return est, metrics

def grid_search_sgd_regression(feature_array, labels, test_size=0.2):
    """
    Performs grid search for hyperparameter tuning of SGD regression.

    Args:
        feature_array (np.ndarray):     Array of features of shape (samples, features).
        labels (np.ndarray):            Array of labels of shape (samples,).
        test_size (float):              Proportion of the dataset to include in the test split.

    Returns:
        sgd_grid_search (GridSearchCV): Fitted grid search object.
    """

    model, _ = sgd_regression(feature_array, labels, test_size, RANDOM_SEED)

    scaler = StandardScaler()
    feature_array = scaler.fit_transform(feature_array)

    parameters = [
        {
            'sgdregressor__max_iter':[100000, 1000000], 
            'sgdregressor__tol':[1e-10, 1e-3],
            'sgdregressor__eta0':[0.001, 0.01]
        }
    ]

    sgd_grid_search = GridSearchCV(
        model,
        parameters,
        cv=10
    )

    sgd_grid_search.fit(feature_array, labels)

    return sgd_grid_search

def monte_carlo_feature_selection(feature_table, labels, data_dir, n_iterations=100, test_size=0.2):
    """
    Performs Monte Carlo feature selection on the given feature table and labels. 

    Args:
        feature_table (pd.DataFrame):   DataFrame containing the features and labels.
        labels (np.ndarray):            Array of labels of shape (samples,).
        data_dir (str):                 Directory to save the optimal feature table.
        n_iterations (int):             Number of iterations to run the Monte Carlo simulation.
        test_size (float):              Proportion of the dataset to include in the test split.

    Returns:
        feature_table_optimal (pd.DataFrame): DataFrame containing the optimal features and labels.
        feature_names (list):           List of feature names selected based on the best performing model.
        labels (np.ndarray):            Array of labels corresponding to the selected features.
    """

    feature_array = feature_table.drop(columns=['Label']).values
    labels = feature_table['Label'].values

    best_features = []
    best_score = np.inf
    best_params = None

    for i in range(n_iterations):
        top_n = np.random.randint(1, feature_array.shape[1] + 1)
        selected_indices = np.random.choice(feature_array.shape[1], top_n, replace=False)
        selected_features = feature_array[:, selected_indices]
        
        # I have succesfully designed the least time optimal feature selection 
        # algorithm here. But I can just run this overnight so it doesn't matter.
        clf = grid_search_sgd_regression(
            selected_features, labels, test_size=test_size
        )

        model, metrics = sgd_regression(
            selected_features, labels, test_size=test_size, random_state=2,
            eta0=clf.best_params_['sgdregressor__eta0'],
            max_iter=clf.best_params_['sgdregressor__max_iter'],
            tol=clf.best_params_['sgdregressor__tol']
        )

        if metrics['mae'] < best_score:
            best_score = metrics['mae']
            best_features = selected_indices
            best_params = clf.best_params_

        if (i + 1) % max(1, n_iterations // 10) == 0:
            print(f"Iteration {i+1} / {n_iterations}")

    feature_table_optimal = feature_table.iloc[:, best_features].copy()
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal,
        data_dir,
        "feature_sgd_monte_carlo.csv"
    )

    # save best params as a json file
    best_params_file = os.path.join(data_dir, "sgd_best_params.json")
    with open(best_params_file, "w") as f:
        json.dump(best_params, f)

    return feature_table_optimal