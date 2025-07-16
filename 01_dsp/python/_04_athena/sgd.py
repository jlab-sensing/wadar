import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.dataset import bulk_density_to_label
from sklearn.linear_model import SGDClassifier, SGDRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score
import pandas as pd
import numpy as np
from sklearn.model_selection import GridSearchCV

def sgd_regression(dataset_dir, feature_file_name, test_size, random_state,
                   eta0=0.001, 
                   max_iter=1000000,
                   tol=1e-10):
    
    data = pd.read_csv(dataset_dir + '/' + feature_file_name)
    X = data.drop(columns=['label']).values
    y = data['label'].values

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=test_size, random_state=random_state)

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
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    metrics = {'mse': mse, 'r2': r2}

    return est, metrics

def grid_search_sgd_regression(dataset_dir, feature_file_name, test_size=0.2, random_state=42):
    """
    Placeholder for grid search implementation.
    Currently, it just calls sgd_regression with default parameters.
    """

    data = pd.read_csv(dataset_dir + '/' + feature_file_name)
    X = data.drop(columns=['label']).values
    y = data['label'].values

    model, _ = sgd_regression(dataset_dir, feature_file_name, test_size, random_state)

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

    sgd_grid_search.fit(X, y)

    return sgd_grid_search

def sgd_classification(dataset_dir, feature_file_name, test_size, random_state,
                      eta0=0.01,
                      max_iter=100000,
                      tol=0.001):
    data = pd.read_csv(dataset_dir + '/' + feature_file_name)
    X = data.drop(columns=['label']).values
    y = data['label'].values

    y_labels = []
    for i, label in enumerate(y):
        y_labels.append(bulk_density_to_label(label))

    X_train, X_test, y_train, y_test = train_test_split(X, y_labels, test_size=test_size, random_state=random_state)

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

def grid_search_sgd_classification(dataset_dir, feature_file_name, test_size=0.2, random_state=42):
    """
    Grid search for SGDClassifier.
    """
    data = pd.read_csv(dataset_dir + '/' + feature_file_name)
    X = data.drop(columns=['label']).values
    y = data['label'].values


    y_labels = []
    for i, label in enumerate(y):
        y_labels.append(bulk_density_to_label(label))

    model, _ = sgd_classification(dataset_dir, feature_file_name, test_size, random_state)

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

    sgd_grid_search.fit(X, y_labels)

    return sgd_grid_search