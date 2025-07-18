# Use of linear regression to predict the bulk density of soil based on the given features.

import numpy as np
from sklearn.discriminant_analysis import StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, r2_score
import pandas as pd
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import RidgeCV
from sklearn.pipeline import Pipeline
from _03_hephaestus import feature_tools

def polynomial_regression(feature_array, labels, test_size=0.2, random_state=21, degree=1):

    X = feature_array
    y = labels

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=test_size, random_state=random_state
    )

    alphas = np.logspace(-6, 3, 100)

    model = Pipeline([          # This is cool I no longer need to save the scaler
        ('poly', PolynomialFeatures(degree=degree, include_bias=False)),
        ('scaler', StandardScaler()),
        ('ridge', RidgeCV(alphas=alphas, cv=5)) # adds L2 regularization to punish
                                                # large coefficients and control overfitting.
                                                # cross validation is used to select the best alpha
    ])

    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    y_pred_train = model.predict(X_train)
    mae_train = mean_absolute_error(y_train, y_pred_train)
    r2_train = r2_score(y_train, y_pred_train)

    return model, {'mse': mae, 'r2': r2, 'mae_train': mae_train, 'r2_train': r2_train}

def monte_carlo_feature_selection(feature_table, labels, data_dir, n_iterations=100, test_size=0.2):
    """
    Test different feature sets using Monte Carlo simulation to determine the
    best feature set for linear regression.
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
            selected_features, labels, test_size=test_size, degree=1
        )

        if metrics['mae_train'] < best_score:
            best_score = metrics['mae_train']
            best_features = selected_indices

        if (i + 1) % max(1, n_iterations // 10) == 0:
            print(f"Iteration {i+1} / {n_iterations}")

    feature_table_optimal = feature_table.iloc[:, best_features]
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal, data_dir, "feature_linear_regression_monte_carlo.csv"
    )

    return feature_array[:, best_features], feature_table.columns[best_features].tolist(), labels