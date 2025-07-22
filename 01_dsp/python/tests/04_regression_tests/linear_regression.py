import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))# https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _04_athena import regression
from _06_hermes.parameters import num2label
from _05_apollo import viz_tools
import time
import pandas as pd
from _06_hermes.logger import update_results


if __name__ == "__main__":

    VIZ = True  # Set to True to visualize features
    MONTE_CARLO = True
    
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"
    test_size = 0.2

    # If making a new feature set,
    # hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    # X, y = hydros.X, hydros.y
    # hephaestus_features = feature_tools.FeatureTools(X)
    # feature_table = hephaestus_features.feature_full_monty(y, dataset_dir)

    # If using an existing feature set,
    feature_table, _, _, labels = feature_tools.load_feature_table(
        dataset_dir, feature_file_name)

    print("Polynomial Regression Model")
    print()

    if not MONTE_CARLO:

        for degree in [1, 2, 3]:

            _, feature_array, _, labels = feature_tools.load_feature_table(
                dataset_dir, f"feature_linear_regression_{degree}_monte_carlo.csv")

            poly_model, poly_metrics = regression.polynomial_regression(
                feature_array, labels, degree=degree, kfold_splits=5
            )
            print("Model Metrics:", poly_metrics)

            mae = poly_metrics["mae"]
            r2 = poly_metrics["r2"]
            accuracy = poly_metrics["accuracy"]
            inference_time = poly_metrics["inference_time"]

            model_name = f"Regression Degree {degree}"

            update_results(model_name, mae, accuracy, inference_time, dataset_dir)
            
            y_pred = poly_model.predict(feature_array).flatten()

            if VIZ:
                # viz_tools.plot_confusion_matrix(
                #     y_labels=[num2label(label) for label in labels],
                #     y_pred=[num2label(pred) for pred in y_pred]
                # )
                viz_tools.plot_regression(labels, y_pred.flatten())
        
        if VIZ:
            plt.show()

    else:

        n_iterations = 1000

        print("Monte Carlo Feature Selection:")

        for degree in [1, 2, 3]:

            print(f"Degree {degree} Monte Carlo Feature Selection")

            mc_results = regression.monte_carlo_regression_feature_selection(
                feature_table, labels, dataset_dir, degree=degree, n_iterations=n_iterations
            )

            feature_table, feature_array, _, labels = feature_tools.load_feature_table(
                dataset_dir, f"feature_linear_regression_{degree}_monte_carlo.csv"
            )
            poly_model, poly_metrics = regression.polynomial_regression(
                feature_array, labels, degree=degree, kfold_splits=5
            )
            print("Model Metrics:", poly_metrics)

            if VIZ:
                viz_tools.plot_regression(
                    labels, poly_model.predict(feature_array).flatten()
                )

        if VIZ:
            plt.show()
