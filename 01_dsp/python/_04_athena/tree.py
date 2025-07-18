from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score
import numpy as np
from sklearn.ensemble import RandomForestRegressor
import pandas as pd



def train_decision_tree_model(feature_array, labels, test_size=0.2, random_state=21, max_depth=5):
    """
    Trains a Decision Tree Regressor on the provided features and labels.
    """

    X_train, X_test, y_train, y_test = train_test_split(feature_array, labels, test_size=test_size, random_state=random_state)

    model = DecisionTreeRegressor(max_depth=max_depth, random_state=random_state)
    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    return model, {'mse': mse, 'r2': r2}

def train_random_forest_model(feature_array, labels, test_size=0.2, random_state=21, n_estimators=100):
    """
    Trains a Random Forest Regressor on the provided features and labels.
    """

    X_train, X_test, y_train, y_test = train_test_split(feature_array, labels, test_size=test_size, random_state=random_state)

    model = RandomForestRegressor(n_estimators=n_estimators, random_state=random_state)
    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    return model, {'mse': mse, 'r2': r2}
