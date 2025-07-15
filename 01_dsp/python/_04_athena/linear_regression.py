# Use of linear regression to predict the bulk density of soil based on the given features.

import numpy as np
from sklearn.discriminant_analysis import StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score
import pandas as pd

def model_linear_regression(dataset_dir, feature_file_name, test_size=0.2, random_state=42):
    """
    Loads feature vectors from a saved .npz file and trains a linear regression model
    to predict the target variable. Pretty basic linear regression
    implementation using scikit-learn.
    """

    data = pd.read_csv(dataset_dir + '/' + feature_file_name)
    X = data.drop(columns=['label']).values
    y = data['label'].values

    # Scale features if necessary
    scaler = StandardScaler()
    X = scaler.fit_transform(X)

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=test_size, random_state=random_state
    )

    model = LinearRegression()
    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    return model, {'mse': mse, 'r2': r2}