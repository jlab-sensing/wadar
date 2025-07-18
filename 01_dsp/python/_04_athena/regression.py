# Use of linear regression to predict the bulk density of soil based on the given features.

import numpy as np
from sklearn.discriminant_analysis import StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score
import pandas as pd
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import RidgeCV
from sklearn.pipeline import Pipeline

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
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    y_pred_train = model.predict(X_train)
    mse_train = mean_squared_error(y_train, y_pred_train)
    r2_train = r2_score(y_train, y_pred_train)

    return model, {'mse': mse, 'r2': r2, 'mse_train': mse_train, 'r2_train': r2_train}
