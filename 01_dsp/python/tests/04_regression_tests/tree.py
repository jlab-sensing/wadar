import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))# https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir

sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _04_athena import tree
from sklearn.tree import plot_tree
import pandas as pd

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

    # Train Decision Tree model
    model, metrics = tree.train_decision_tree_model(
        feature_array,
        labels,
        test_size=test_size,
        max_depth=5
    )

    print("Trained Decision Tree model:", model)
    print("Metrics:", metrics)

    plt.figure(figsize=(12, 8))
    plot_tree(
        model,
        feature_names=feature_names,
        filled=True,
        precision=3,
        fontsize=10
    )
    plt.title("Decision Tree Structure")

    # Train Random Forest model
    model_rf, metrics_rf = tree.train_random_forest_model(
        feature_array,
        labels,
        test_size=test_size,
        n_estimators=100
    )

    print("Trained Random Forest model:", model_rf)
    print("Metrics (Random Forest):", metrics_rf)

    plt.figure(figsize=(12, 8))
    plot_tree(
        model,
        feature_names=feature_names,
        filled=True,
        precision=3,
        fontsize=10
    )
    plt.title("Decision Tree Structure")
    plt.show()