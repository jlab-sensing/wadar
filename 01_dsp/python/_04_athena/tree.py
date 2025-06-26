from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score
import numpy as np
from sklearn.ensemble import RandomForestRegressor

def train_decision_tree_model(dataset_dir, target, test_size=0.2, random_state=42, max_depth=5):
    """
    Loads engineered features from an .npz file and trains a Decision Tree Regressor.

    Parameters:
        dataset_dir (str): Path to directory containing features.npz
        target (np.ndarray): Target values (e.g. soil compaction), shape (N,)
        test_size (float): Proportion of data for testing
        max_depth (int): Maximum depth of decision tree

    Returns:
        model: Trained DecisionTreeRegressor
        metrics: Dictionary with MSE and R²
    """
    data = np.load(f"{dataset_dir}/features.npz")
    X = np.column_stack([data[key] for key in data.keys()])

    assert len(X) == len(target), "Target length must match number of samples."

    X_train, X_test, y_train, y_test = train_test_split(X, target, test_size=test_size, random_state=random_state)

    model = DecisionTreeRegressor(max_depth=max_depth, random_state=random_state)
    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    print(f"Decision Tree (depth={max_depth}) → MSE: {mse:.4f}, R²: {r2:.4f}")
    return model, {'mse': mse, 'r2': r2}

def train_random_forest_model(dataset_dir, target, test_size=0.2, random_state=42, n_estimators=100):
    """
    Trains a Random Forest Regressor on saved features.

    Parameters:
        dataset_dir (str): Path to directory containing features.npz
        target (np.ndarray): Target values (e.g. soil compaction), shape (N,)
        test_size (float): Proportion of data for testing
        max_depth (int): Maximum depth of decision tree

    Returns:
        model: Trained DecisionTreeRegressor
        metrics: Dictionary with MSE and R²
    """
    data = np.load(f"{dataset_dir}/features.npz")
    X = np.column_stack([data[key] for key in data.keys()])

    X_train, X_test, y_train, y_test = train_test_split(X, target, test_size=test_size, random_state=random_state)

    model = RandomForestRegressor(n_estimators=n_estimators, random_state=random_state)
    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    print(f"Random Forest ({n_estimators} trees) → MSE: {mse:.4f}, R²: {r2:.4f}")
    return model, {'mse': mse, 'r2': r2}
