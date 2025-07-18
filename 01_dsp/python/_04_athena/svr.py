import json
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from sklearn.svm import SVR
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler, MinMaxScaler
from sklearn.model_selection import GridSearchCV
from _03_hephaestus import feature_tools

def svr_regression(feature_array, labels, test_size=0.2, C=1.0, gamma='scale', epsilon=0.1):
    X_train, X_test, y_train, y_test = train_test_split(feature_array, labels, test_size=test_size)

    clf = make_pipeline(StandardScaler(), SVR(kernel='rbf', 
                                              C=C, 
                                              gamma=gamma, 
                                              epsilon=epsilon)
                                              )
    clf.fit(X_train, y_train)

    y_pred = clf.predict(X_test)
    mae = mean_absolute_error(y_test, y_pred)
    r2 = clf.score(X_test, y_test)
    metrics = {'mae': mae, 'r2': r2}
    return clf, metrics

def tune_svr(feature_array, labels):

    # Sourced from https://medium.com/@jayedakhtar96/learn-how-to-implement-svr-for-real-world-data-using-python-sklearn-and-hyperparameter-tuning-7b12a2df9074
    param_grid = {
        'C': [0.1, 1, 10, 100],
        'gamma': [0.01, 0.1, 1, 'scale'],
        'epsilon': [0.01, 0.1, 0.5]
    }

    scaler = StandardScaler()
    feature_array = scaler.fit_transform(feature_array)

    clf = SVR(kernel='rbf')

    grid_search = GridSearchCV(
        clf,
        param_grid,
        cv=10
    )
    grid_search.fit(feature_array, labels)

    return grid_search.best_params_

def monte_carlo_svr_feature_selection(feature_table, labels, data_dir, n_iterations=100, test_size=0.2):
    """
    Test different feature sets using Monte Carlo simulation to determine the
    best feature set for SVR.
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
        
        best_params = tune_svr(selected_features, labels)
        model, metrics = svr_regression(
            selected_features,
            labels,
            test_size=test_size,
            C=best_params['C'],
            gamma=best_params['gamma'],
            epsilon=best_params['epsilon']
        )

        if metrics['mae'] < best_score:
            best_score = metrics['mae']
            best_features = selected_indices
            best_params = best_params

        if (i + 1) % max(1, n_iterations // 10) == 0:
            print(f"Iteration {i + 1}/{n_iterations} - Best MAE: {best_score:.4f}")

    feature_table_optimal = feature_table.iloc[:, best_features].copy()
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal,
        data_dir,
        "feature_svr_monte_carlo.csv"
    )

    # save best params as a json file
    best_params_file = os.path.join(data_dir, "svr_best_params.json")
    with open(best_params_file, "w") as f:
        json.dump(best_params, f)

    return feature_table_optimal