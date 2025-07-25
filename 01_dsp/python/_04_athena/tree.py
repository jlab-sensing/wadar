from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, r2_score
import numpy as np
from sklearn.ensemble import RandomForestRegressor
import pandas as pd
from _03_hephaestus import feature_tools
from xgboost import XGBRegressor
import time
from _06_hermes.parameters import num2label, RANDOM_SEED, KFOLD_SPLITS
from sklearn.model_selection import KFold

n_estimators = 100

def train_random_forest(feature_array, labels, n_estimators=n_estimators, kfold_splits=KFOLD_SPLITS):
    """
    Trains a Random Forest Regressor on the provided features and labels. 

    Args:
        feature_array (np.ndarray):    Array of features of shape (samples, features).
        labels (np.ndarray):           Array of labels of shape (samples,).
        n_estimators (int):            Number of trees in the forest.
        kfold_splits (int):            Number of K-Fold splits.

    Returns:
        model (RandomForestRegressor): Trained Random Forest model.
        dict:                          Dictionary containing evaluation metrics such as MAE, RMSE, R2, accuracy, training time, and inference time.
    """
    kf = KFold(n_splits=kfold_splits, shuffle=True, random_state=RANDOM_SEED)
    maes, rmses, r2s, accuracies, inference_times, training_times = [], [], [], [], [], []
    models = []

    for train_index, test_index in kf.split(feature_array):
        X_train, X_test = feature_array[train_index], feature_array[test_index]
        y_train, y_test = labels[train_index], labels[test_index]

        train_start = time.time()
        model = RandomForestRegressor(n_estimators=n_estimators, random_state=RANDOM_SEED)
        model.fit(X_train, y_train)
        training_time = time.time() - train_start
        models.append(model)

        time_start = time.time()
        y_pred = model.predict(X_test)
        inference_time = time.time() - time_start

        mae = mean_absolute_error(y_test, y_pred)
        rmse = np.sqrt(np.mean((y_test - y_pred) ** 2))
        r2 = r2_score(y_test, y_pred)
        y_labels = [num2label(label) for label in y_test]
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

        maes.append(mae)
        rmses.append(rmse)
        r2s.append(r2)
        accuracies.append(accuracy)
        inference_times.append(inference_time)
        training_times.append(training_time)

    model = RandomForestRegressor(n_estimators=n_estimators, random_state=RANDOM_SEED)
    model.fit(feature_array, labels)

    return models[-1], {
        'mae': np.mean(maes),
        'rmse': np.mean(rmses),
        'r2': np.mean(r2s),
        'accuracy': np.mean(accuracies),
        'training_time': np.mean(training_times),
        'inference_time': np.mean(inference_times)
    }

def monte_carlo_random_tree_feature_selection(feature_table, labels, data_dir, n_iterations=100):
    """
    Performs Monte Carlo feature selection on the given feature table and labels. 

    Args:
        feature_table (pd.DataFrame):   DataFrame containing the features and labels.
        labels (np.ndarray):            Array of labels of shape (samples,).
        data_dir (str):                 Directory to save the feature table.
        n_iterations (int):             Number of iterations for the Monte Carlo simulation.
        test_size (float):              Proportion of the dataset to include in the test split.

    Returns:
        np.ndarray:                     Array of selected features after Monte Carlo simulation.
        list:                           List of selected feature names.
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
        
        model_rf, metrics_rf = train_random_forest(
            selected_features,
            labels,
            n_estimators=100
        )

        if metrics_rf['mae'] < best_score:
            print(f"Improved MAE from {best_score:.4f} to {metrics_rf['mae']:.4f} at iteration {i+1}")

        if metrics_rf['mae'] < best_score:
            best_score = metrics_rf['mae']
            best_features = selected_indices

    feature_table_optimal = feature_table.copy().iloc[:, best_features]
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal, data_dir, "models/feature_random_forest_monte_carlo.csv"
    )

    return feature_array[:, best_features], feature_table.columns[best_features].tolist(), labels

def train_gradient_boosted_tree(feature_array, labels, n_estimators=n_estimators, kfold_splits=5):
    """
    Trains an XGBoost Regressor on the provided features and labels using K-Fold cross-validation.

    Args:
        feature_array (np.ndarray):    Array of features of shape (samples, features).
        labels (np.ndarray):           Array of labels of shape (samples,).
        n_estimators (int):            Number of boosting rounds.
        kfold_splits (int):            Number of K-Fold splits.

    Returns:
        model (XGBRegressor):          Last trained XGBoost model.
        dict:                          Dictionary containing averaged evaluation metrics (MAE, RMSE, R2, accuracy, training time, inference time).
    """

    kf = KFold(n_splits=kfold_splits, shuffle=True, random_state=RANDOM_SEED)
    maes, rmses, r2s, accuracies, inference_times, training_times = [], [], [], [], [], []
    models = []

    for train_index, test_index in kf.split(feature_array):
        X_train, X_test = feature_array[train_index], feature_array[test_index]
        y_train, y_test = labels[train_index], labels[test_index]

        train_start = time.time()
        model = XGBRegressor(n_estimators=n_estimators, random_state=RANDOM_SEED, verbosity=0)
        model.fit(X_train, y_train)
        training_time = time.time() - train_start
        models.append(model)

        time_start = time.time()
        y_pred = model.predict(X_test)
        inference_time = time.time() - time_start

        mae = mean_absolute_error(y_test, y_pred)
        rmse = np.sqrt(np.mean((y_test - y_pred) ** 2))
        r2 = r2_score(y_test, y_pred)
        y_labels = [num2label(label) for label in y_test]
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

        maes.append(mae)
        rmses.append(rmse)
        r2s.append(r2)
        accuracies.append(accuracy)
        inference_times.append(inference_time)
        training_times.append(training_time)

    model = XGBRegressor(n_estimators=n_estimators, random_state=RANDOM_SEED, verbosity=0)
    model.fit(feature_array, labels)

    return models[-1], {
        'mae': np.mean(maes),
        'rmse': np.mean(rmses),
        'r2': np.mean(r2s),
        'accuracy': np.mean(accuracies),
        'training_time': np.mean(training_times),
        'inference_time': np.mean(inference_times)
    }

def monte_carlo_gradient_boosted_tree_feature_selection(feature_table, labels, data_dir, n_iterations=100, test_size=0.2):
    """
    Performs Monte Carlo feature selection on the given feature table and labels. 

    Args:
        feature_table (pd.DataFrame):   DataFrame containing the features and labels.
        labels (np.ndarray):            Array of labels of shape (samples,).
        data_dir (str):                 Directory to save the feature table.
        n_iterations (int):             Number of iterations for the Monte Carlo simulation.
        test_size (float):              Proportion of the dataset to include in the test split.

    Returns:
        np.ndarray:                     Array of selected features after Monte Carlo simulation.
        list:                           List of selected feature names.
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
        
        model, metrics = train_gradient_boosted_tree(
            selected_features,
            labels,
            n_estimators=100
        )

        if metrics['mae'] < best_score:
            print(f"Improved MAE from {best_score:.4f} to {metrics['mae']:.4f} at iteration {i+1}")

        if metrics['mae'] < best_score:
            best_score = metrics['mae']
            best_features = selected_indices

    feature_table_optimal = feature_table.copy().iloc[:, best_features]
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal, data_dir, "models/feature_gradient_boosted_tree_monte_carlo.csv"
    )

    return feature_array[:, best_features], feature_table.columns[best_features].tolist(), labels

# def train_decision_tree_model(feature_array, labels, max_depth=5):
#     """
#     Trains a Decision Tree Regressor on the provided features and labels. I don't 
#     use this because they seem to operate objectively worse than Random Forests,
#     with the only benefit being that they are more interpretable. Random forests
#     are more interpretable than the other models anyway.
#     """

#     X_train, X_test, y_train, y_test = train_test_split(feature_array, labels, test_size=test_size, random_state=RANDOM_SEED)

#     model = DecisionTreeRegressor(max_depth=max_depth, random_state=RANDOM_SEED)
#     model.fit(X_train, y_train)

#     time_start = time.time()
#     y_pred = model.predict(X_test)
#     inference_time = time.time() - time_start

#     mae = mean_absolute_error(y_test, y_pred)
#     r2 = r2_score(y_test, y_pred)

#     y_labels = [num2label(label) for label in y_test]
#     accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

#     return model, {'mae': mae, 'r2': r2, 'accuracy': accuracy, 'inference_time': inference_time}