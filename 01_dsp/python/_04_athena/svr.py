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
import time
from _06_hermes.parameters import num2label, RANDOM_SEED, KFOLD_SPLITS
from sklearn.model_selection import KFold

def svr_regression(feature_array, labels, C=1.0, gamma='scale', epsilon=0.1, k_folds=KFOLD_SPLITS):
    """
    Performs Support Vector Regression (SVR) with K-Fold cross-validation.

    Args:
        feature_array (np.ndarray):    Array of features of shape (samples, features).
        labels (np.ndarray):           Array of labels of shape (samples,).
        C (float):                     Regularization parameter.
        gamma (str or float):          Kernel coefficient for 'rbf', 'poly', and 'sigmoid'.
        epsilon (float):               Epsilon in the epsilon-SVR model.
        k_folds (int):                 Number of folds for K-Fold cross-validation.

    Returns:
        clf (SVR):                     Trained SVR model (last fold).
        metrics (dict):                Dictionary containing mean evaluation metrics across folds.
    """

    maes, r2s, accuracies, inference_times = [], [], [], []

    kf = KFold(n_splits=k_folds, shuffle=True, random_state=RANDOM_SEED)
    for train_idx, test_idx in kf.split(feature_array):
        X_train, X_test = feature_array[train_idx], feature_array[test_idx]
        y_train, y_test = labels[train_idx], labels[test_idx]

        clf = make_pipeline(StandardScaler(), SVR(kernel='rbf', C=C, gamma=gamma, epsilon=epsilon))
        clf.fit(X_train, y_train)

        time_start = time.time()
        y_pred = clf.predict(X_test)
        inference_time = time.time() - time_start

        mae = mean_absolute_error(y_test, y_pred)
        r2 = clf.score(X_test, y_test)
        y_labels = [num2label(label) for label in y_test]
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

        maes.append(mae)
        r2s.append(r2)
        accuracies.append(accuracy)
        inference_times.append(inference_time)
    # Return last trained model and mean metrics
    metrics = {
        'mae': np.mean(maes),
        'r2': np.mean(r2s),
        'accuracy': np.mean(accuracies),
        'inference_time': np.mean(inference_times)
    }
    return clf, metrics

def tune_svr(feature_array, labels):
    """
    Tune the hyperparameters of the SVR model using GridSearchCV. Sourced from
    https://medium.com/@jayedakhtar96/learn-how-to-implement-svr-for-real-world-data-using-python-sklearn-and-hyperparameter-tuning-7b12a2df9074
    
    Args:
        feature_array (np.ndarray):    Array of features of shape (samples, features).
        labels (np.ndarray):           Array of labels of shape (samples,).
        
    Returns:
        dict:                          Dictionary containing the best hyperparameters for the SVR model.
    """

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

def monte_carlo_svr_feature_selection(feature_table, labels, data_dir, n_iterations=100,):
    """
    Performs Monte Carlo feature selection on the given feature table and labels. 

    Args:
        feature_table (pd.DataFrame):   DataFrame containing the features and labels.
        labels (np.ndarray):            Array of labels of shape (samples,).
        data_dir (str):                 Directory to save the feature table.
        n_iterations (int):             Number of iterations for the Monte Carlo simulation.
        test_size (float):              Proportion of the dataset to include in the test split.

    Returns:
        pd.DataFrame:                  DataFrame containing the optimal feature set after Monte Carlo simulation.
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
            C=best_params['C'],
            gamma=best_params['gamma'],
            epsilon=best_params['epsilon']
        )

        if metrics['mae'] < best_score:
            print(f"Improved MAE from {best_score:.4f} to {metrics['mae']:.4f} at iteration {i+1}")

        if metrics['mae'] < best_score:
            best_score = metrics['mae']
            best_features = selected_indices
            best_params = best_params

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