import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools
import _04_athena.svr as svr
import pandas as pd
from _05_apollo import viz_tools
import json
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
    feature_table, feature_array, _, labels = feature_tools.load_feature_table(
        dataset_dir, feature_file_name)

    if not MONTE_CARLO:

        _, feature_array, feature_names, labels = feature_tools.load_feature_table(
            dataset_dir, "feature_svr_monte_carlo.csv")

        best_params = svr.tune_svr(feature_array, labels)
        model, metrics = svr.svr_regression(feature_array, labels,
                                            C=best_params['C'],
                                            gamma=best_params['gamma'],
                                            epsilon=best_params['epsilon'])

        print(f"SVR Model Metrics: {metrics}")

        mae = metrics['mae']
        r2 = metrics['r2']
        accuracy = metrics['accuracy']
        inference_time = metrics['inference_time']

        model_name = "SVR"

        update_results(model_name, mae, accuracy, inference_time, dataset_dir)

        viz_tools.plot_regression(
            labels, model.predict(feature_array).flatten()
        )
        plt.show()

    else:

        print("Running Monte Carlo feature selection for SVR...")

        feature_table = svr.monte_carlo_svr_feature_selection(
            feature_table, labels, dataset_dir, n_iterations=100
        )
        
        feature_table, feature_array, feature_names, labels = feature_tools.load_feature_table(
            dataset_dir, "feature_svr_monte_carlo.csv"
        )
        model, metrics = svr.svr_regression(
            feature_array, labels
        )

        print("Metrics (SVR):", metrics)

        if VIZ:
            viz_tools.plot_regression(
                labels, model.predict(feature_array).flatten()
            )

            plt.show()