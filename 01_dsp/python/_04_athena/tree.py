from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, r2_score
import numpy as np
from sklearn.ensemble import RandomForestRegressor
import pandas as pd
from _03_hephaestus import feature_tools
from xgboost import XGBRegressor
import time
from _06_hermes.parameters import num2label, RANDOM_SEED

def train_decision_tree_model(feature_array, labels, test_size=0.2, max_depth=5):
    """
    Trains a Decision Tree Regressor on the provided features and labels. I don't 
    use this because they seem to operate objectively worse than Random Forests,
    with the only benefit being that they are more interpretable. Random forests
    are more interpretable than the other models anyway.
    """

    X_train, X_test, y_train, y_test = train_test_split(feature_array, labels, test_size=test_size, random_state=random_state)

    model = DecisionTreeRegressor(max_depth=max_depth, random_state=RANDOM_SEED)
    model.fit(X_train, y_train)

    time_start = time.time()
    y_pred = model.predict(X_test)
    inference_time = time.time() - time_start

    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    y_labels = [num2label(label) for label in y_test]
    accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

    return model, {'mae': mae, 'r2': r2, 'accuracy': accuracy, 'inference_time': inference_time}

def train_random_forest(feature_array, labels, test_size=0.2, random_state=21, n_estimators=100):
    """
    Trains a Random Forest Regressor on the provided features and labels.
    """

    X_train, X_test, y_train, y_test = train_test_split(feature_array, labels, test_size=test_size, random_state=RANDOM_SEED)

    model = RandomForestRegressor(n_estimators=n_estimators, random_state=RANDOM_SEED)
    model.fit(X_train, y_train)

    time_start = time.time()
    y_pred = model.predict(X_test)
    inference_time = time.time() - time_start

    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    y_labels = [num2label(label) for label in y_test]
    accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

    return model, {'mae': mae, 'r2': r2, 'accuracy': accuracy, 'inference_time': inference_time}

def monte_carlo_random_tree_feature_selection(feature_table, labels, data_dir, n_iterations=100, test_size=0.2):
    """
    Test different feature sets using Monte Carlo simulation to determine the
    best feature set for random forest. Not sure if random forests are better,
    but they are more robust to overfitting, which is something I'm concerned about
    with our smaller than ideal dataset.
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
            test_size=test_size,
            n_estimators=100
        )

        if metrics_rf['mae'] < best_score:
            best_score = metrics_rf['mae']
            best_features = selected_indices

        if (i + 1) % max(1, n_iterations // 10) == 0:
            print(f"Iteration {i+1} / {n_iterations}")

    feature_table_optimal = feature_table.iloc[:, best_features]
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal, data_dir, "feature_random_forest_monte_carlo.csv"
    )

    return feature_array[:, best_features], feature_table.columns[best_features].tolist(), labels

def train_gradient_boosted_tree(feature_array, labels, test_size=0.2, n_estimators=100):
    """
    Trains a XGBoost Regressor on the provided features and labels.
    """

    X_train, X_test, y_train, y_test = train_test_split(feature_array, 
                                                        labels, 
                                                        test_size=test_size, 
                                                        random_state=RANDOM_SEED)
    
    # maybe I should perform a grid search but it performs well enough without it
    model = XGBRegressor(n_estimators=n_estimators)
    model.fit(X_train, y_train)

    time_start = time.time()
    y_pred = model.predict(X_test)
    inference_time = time.time() - time_start

    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)
    y_labels = [num2label(label) for label in y_test]
    accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

    return model, {'mae': mae, 'r2': r2, 'accuracy': accuracy, 'inference_time': inference_time}

def monte_carlo_gradient_boosted_tree_feature_selection(feature_table, labels, data_dir, n_iterations=100, test_size=0.2):
    """
    Test different feature sets using Monte Carlo simulation to determine the
    best feature set for gradient boosted trees.
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
            test_size=test_size,
            n_estimators=100
        )

        if metrics['mae'] < best_score:
            best_score = metrics['mae']
            best_features = selected_indices

        if (i + 1) % max(1, n_iterations // 10) == 0:
            print(f"Iteration {i+1} / {n_iterations} - Best MAE: {best_score:.4f}")

    feature_table_optimal = feature_table.iloc[:, best_features]
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal, data_dir, "feature_gradient_boosted_tree_monte_carlo.csv"
    )

    return feature_array[:, best_features], feature_table.columns[best_features].tolist(), labels