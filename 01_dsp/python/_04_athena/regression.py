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
import time
from _06_hermes.parameters import num2label, RANDOM_SEED

def polynomial_regression(feature_array, labels, test_size=0.2, degree=1):
    """
    Performs polynomial regression on the given feature array and labels. 

    Args:
        feature_array (np.ndarray):    Array of features of shape (samples, features).
        labels (np.ndarray):           Array of labels of shape (samples,).
        test_size (float):             Proportion of the dataset to include in the test split.
        degree (int):                  Degree of the polynomial features to be generated.

    Returns:
        model (Pipeline):              Trained polynomial regression model.
        dict:                          Dictionary containing evaluation metrics such as MAE, R2, accuracy, and inference time.
    """

    X = feature_array
    y = labels

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=test_size, random_state=RANDOM_SEED
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

    time_start = time.time()
    y_pred = model.predict(X_test)
    inference_time = time.time() - time_start

    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    y_labels = [num2label(label) for label in y_test]
    accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

    return model, {
        "mae": mae,
        "r2": r2,
        "accuracy": accuracy,
        "inference_time": inference_time
    }

def monte_carlo_feature_selection(feature_table, labels, data_dir, n_iterations=100, test_size=0.2):
    """
    Test different feature sets using Monte Carlo simulation to determine the best performing features.

    Args:
        feature_table (pd.DataFrame):   DataFrame containing the features and labels.
        labels (np.ndarray):            Array of labels corresponding to the features.
        data_dir (str):                 Directory to save the feature table.
        n_iterations (int):             Number of iterations to run the Monte Carlo simulation.
        test_size (float):              Proportion of the dataset to include in the test split.

    Returns:
        np.ndarray:                     Array of features selected based on the best performing model.
        list:                           List of feature names selected based on the best performing model.
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