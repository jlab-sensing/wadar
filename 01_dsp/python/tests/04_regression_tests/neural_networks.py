import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
import pandas as pd

from _05_apollo import viz_tools
from _03_hephaestus import feature_tools
import tensorflow as tf
from _04_athena.multi_later_percepetron import MultiLaterPercepetron, monte_carlo_mlp_feature_selection
from _04_athena.cnn_models import BabyCNNRegressor

tf.get_logger().setLevel('ERROR')

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features
    
    dataset_dir = "../../data/training-dataset"
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

    # ===================================================

    monte_carlo_mlp_feature_selection(feature_table, labels, dataset_dir, n_iterations=2)

    _, feature_array, feature_names, labels = feature_tools.load_feature_table(
        dataset_dir, "models/feature_mlp_monte_carlo.csv"
    )

    mlp = MultiLaterPercepetron(feature_array, labels)
    model, metrics = mlp.full_monty()

    mae = metrics["mae"]
    rmse = metrics["rmse"]
    r2 = metrics["r2"]
    training_time = metrics["training_time"]
    inference_time = metrics["inference_time"]
    accuracy = metrics["accuracy"]

    print("Cross-Validation Results:")
    print(f"MAE: {mae:.4f}")
    print(f"RMSE: {rmse:.4f}")
    print(f"R2: {r2:.4f}")
    print(f"Training Time: {training_time:.4f} seconds")
    print(f"Inference Time: {inference_time:.4f} seconds")
    print(f"Accuracy: {accuracy:.4f}")