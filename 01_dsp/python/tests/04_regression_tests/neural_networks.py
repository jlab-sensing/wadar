import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
import pandas as pd

from _06_hermes.bulk_density_labels import bulk_density_to_label
from _05_apollo import viz_tools
from _03_hephaestus import feature_tools
import tensorflow as tf
from _04_athena.simple_models import SimpleRegressor
from _04_athena.cnn_models import BabyCNNRegressor

tf.get_logger().setLevel('ERROR')

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features
    
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"
    test_size = 0.2

    # If making a new feature set,
    # hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    # X, y = hydros.X, hydros.y
    # hephaestus_features = feature_tools.FeatureTools(X)
    # feature_table = hephaestus_features.feature_full_monty(y, dataset_dir)

    # If using an existing feature set,
    feature_table, _, _, _ = feature_tools.load_feature_table(
        dataset_dir, feature_file_name)
    
    df_best, mi_scores = feature_tools.mutual_info_minimize_features(feature_table, top_n=10)
    feature_tools.save_feature_table(df_best, dataset_dir, "features_mutual_info.csv")
    _, feature_array, feature_names, labels = feature_tools.load_feature_table(
        dataset_dir, "features_mutual_info.csv")

    # ===================================================

    # regressor = SimpleRegressor(feature_array, labels, test_size=0.2, kfold_splits=5)
    # regressor.train()
    # regressor.evaluate()
    # # regressor.evaluate_classification(bulk_density_to_label, viz_tools)

   # ===================================================

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X = np.abs(hydros.X)
    y = hydros.y

    cnn_cv = BabyCNNRegressor(X, y)
    cnn_cv.cross_validate()
    cnn_cv.train_final_model()
    cnn_cv.evaluate_holdout()
    cnn_cv.plot_predictions()
