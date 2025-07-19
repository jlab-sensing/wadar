import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))# https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _04_athena import regression
from _06_hermes.bulk_density_labels import bulk_density_to_label
from _05_apollo import viz_tools

import pandas as pd

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features
    MONTE_CARLO = False
    
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

        _, feature_array, _, labels = feature_tools.load_feature_table(
            dataset_dir, "feature_linear_regression_monte_carlo")

        print("Degree 1:")
        poly_model, poly_metrics = regression.polynomial_regression(
            feature_array, labels, degree=1, test_size=test_size)
        print("Model Metrics:", poly_metrics)

        y_labels = []
        for i, label in enumerate(labels):
            y_labels.append(bulk_density_to_label(label))

        y_pred = poly_model.predict(feature_array)

        accuracy = np.mean([bulk_density_to_label(pred) == y for pred, y in zip(y_pred, y_labels)])
        print(f"Classification Accuracy: {accuracy:.2f}")
        print()

        if VIZ:
            viz_tools.plot_confusion_matrix(
                y_labels=y_labels,
                y_pred=[bulk_density_to_label(pred) for pred in y_pred]
            )


        print("Degree 2:")
        poly_model, poly_metrics = regression.polynomial_regression(
            feature_array, labels, degree=2, test_size=test_size)
        print("Model Metrics:", poly_metrics)
        print()

        print("Degree 3:")
        poly_model, poly_metrics = regression.polynomial_regression(
            feature_array, labels, degree=3, test_size=test_size)
        print("Model Metrics:", poly_metrics)

        plt.show()

    else:

        print("Monte Carlo Feature Selection:")
        mc_results = regression.monte_carlo_feature_selection(
            feature_table, labels, dataset_dir, n_iterations=1000, test_size=test_size
        )

        feature_table, feature_array, _, labels = feature_tools.load_feature_table(
            dataset_dir, "feature_linear_regression_monte_carlo.csv"
        )
        poly_model, poly_metrics = regression.polynomial_regression(
            feature_array, labels, degree=1, test_size=test_size
        )
        print("Model Metrics:", poly_metrics)
        viz_tools.plot_regression(
            labels, poly_model.predict(feature_array).flatten()
        )
        plt.show()
