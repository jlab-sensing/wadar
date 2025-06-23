# Use of linear regression to predict the bulk density of soil based on the given features.

import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score

def model_linear_regression(dataset_dir, target, test_size=0.2, random_state=42):
    """
    Loads feature vectors from a saved .npz file and trains a linear regression model
    to predict the target variable. Pretty basic linear regression
    implementation using scikit-learn.
    """

    data = np.load(f"{dataset_dir}/features.npz")
    feature_keys = list(data.keys())
    print("Loaded features:", feature_keys)

    X = np.column_stack([data[key] for key in feature_keys])

    X_train, X_test, y_train, y_test = train_test_split(
        X, target, test_size=test_size, random_state=random_state
    )

    model = LinearRegression()
    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    return model, {'mse': mse, 'r2': r2}