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

def sgd_regression(feature_array, labels, test_size, random_state=21,
                   eta0=0.001, 
                   max_iter=1000000,
                   tol=1e-10):
    
    X_train, X_test, y_train, y_test = train_test_split(feature_array, labels, test_size=test_size, random_state=random_state)

    est = make_pipeline(
        StandardScaler(),
        SGDRegressor(
            eta0=eta0,
            max_iter=max_iter,
            tol=tol
        )
    )

    est.fit(X_train, y_train)

    y_pred = est.predict(X_test)
    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    metrics = {'mae': mae, 'r2': r2}

    return est, metrics

def grid_search_sgd_regression(feature_array, labels, test_size=0.2, random_state=21):

    model, _ = sgd_regression(feature_array, labels, test_size, random_state)

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

def sgd_classification(feature_array, labels, test_size, random_state=21,
                      eta0=0.01,
                      max_iter=100000,
                      tol=0.001):

    y_labels = []
    for i, label in enumerate(labels):
        y_labels.append(bulk_density_to_label(label))

    X_train, X_test, y_train, y_test = train_test_split(feature_array, y_labels, test_size=test_size, random_state=random_state)

    est = make_pipeline(
        StandardScaler(),
        SGDClassifier(
            eta0=eta0,
            max_iter=max_iter,
            tol=tol
        )
    )

    est.fit(X_train, y_train)

    y_pred = est.predict(X_test)
    accuracy = np.mean(y_pred == y_test)

    metrics = {'accuracy': accuracy}

    return est, metrics

def grid_search_sgd_classification(feature_array, labels, test_size=0.2, random_state=21):

    y_labels = []
    for i, label in enumerate(labels):
        y_labels.append(bulk_density_to_label(label))

    model, _ = sgd_classification(feature_array, labels, test_size, random_state)

    parameters = [
        {
            'sgdclassifier__max_iter':[100000, 1000000], 
            'sgdclassifier__tol':[1e-10, 1e-3],
            'sgdclassifier__eta0':[0.001, 0.01]
        }
    ]

    sgd_grid_search = GridSearchCV(
        model,
        parameters,
        cv=10
    )

    sgd_grid_search.fit(feature_array, y_labels)

    return sgd_grid_search

def monte_carlo_feature_selection(feature_table, labels, data_dir, n_iterations=100, test_size=0.2):
    """
    Test different feature sets using Monte Carlo simulation to determine the
    best feature set for SGD regression.
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