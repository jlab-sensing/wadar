import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _04_athena import regression

import pandas as pd

def plot_feature_importance(model, feature_names):
    """
    Plots the feature importance from a linear regression model.
    """
    importance = model.coef_
    indices = np.argsort(np.abs(importance))[::-1]

    plt.figure(figsize=(10, 6))
    plt.title("Feature Importance")
    plt.bar(range(len(importance)), importance[indices], align='center')
    plt.xticks(range(len(importance)), np.array(feature_names)[indices], rotation=90)
    plt.xlabel("Features")
    plt.ylabel("Importance")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":

    VIZ = True  # Set to True to visualize features
    
    dataset_dir = "../data/dry-soil-compaction-dataset"
    feature_file_name = "features_selected.csv"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    features = feature_tools.lasso_minimize_features(dataset_dir, X, y)

    # print("Linear Regression Model:")
    # model, metrics = regression.model_linear_regression(dataset_dir, feature_file_name)
    # print("Model Metrics:", metrics)
    # print("Model Coefficients:", model.coef_)
    # print()

    print("Polynomial Regression Model")
    print()

    print("Degree 1:")
    poly_model, poly_metrics = regression.polynomial_regression(
        dataset_dir, feature_file_name, degree=1)
    print("Model Metrics:", poly_metrics)

    print("Degree 2:")
    poly_model, poly_metrics = regression.polynomial_regression(
        dataset_dir, feature_file_name, degree=2)
    print("Model Metrics:", poly_metrics)

    print("Degree 3:")
    poly_model, poly_metrics = regression.polynomial_regression(
        dataset_dir, feature_file_name, degree=3)
    print("Model Metrics:", poly_metrics)
    
    # Doesn't work for polynomial regression
    # if VIZ:
    #     data = pd.read_csv(dataset_dir + '/' + feature_file_name)
    #     feature_names = data.drop(columns=['label']).columns.tolist()
    #     plot_feature_importance(poly_model, feature_names=feature_names)