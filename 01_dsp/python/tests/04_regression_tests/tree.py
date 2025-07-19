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
from _05_apollo import viz_tools

def plot_random_forest(feature_table, model_rf):
    fig, ax = plt.subplots(figsize=(16, 10), dpi=100)
    plot_tree(
            model_rf.estimators_[0],
            feature_names=feature_table.columns[:-1].tolist(),
            filled=True,
            rounded=True,
            precision=3,
            fontsize=5,
            ax=ax,
            proportion=True
        )
    ax.set_title("Random Forest: Example Decision Tree", fontsize=5, fontweight='bold')
    plt.tight_layout()

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features
    MONTE_CARLO = False

    RANDOM_FOREST = False # Set to True to use Random Forest, False for Gradient Boosted Tree
    
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
    
    if not MONTE_CARLO:

        if RANDOM_FOREST:

            _, feature_array, feature_names, labels = feature_tools.load_feature_table(
                dataset_dir, "feature_random_forest_monte_carlo.csv")
            
            # Train Random Forest model
            model_rf, metrics_rf = tree.train_random_forest(
                feature_array,
                labels,
                test_size=test_size,
                n_estimators=100
            )

            print("Trained Random Forest model:", model_rf)
            print("Metrics (Random Forest):", metrics_rf)

            plot_random_forest(feature_table, model_rf)

            viz_tools.plot_regression(
                labels, model_rf.predict(feature_array).flatten()
            )

        else:

            _, feature_array, feature_names, labels = feature_tools.load_feature_table(
                dataset_dir, "feature_gradient_boosted_tree_monte_carlo.csv")

            # Train Gradient Boosted Tree model
            model, metrics = tree.train_gradient_boosted_tree(
                feature_array,
                labels,
                test_size=test_size,
                n_estimators=100
            )

            print("Trained Gradient Boosted Tree model:", model)
            print("Metrics (Gradient Boosted Tree):", metrics)

            viz_tools.plot_regression(
                labels, model.predict(feature_array).flatten()
            )

        plt.show()
    
    else:

        if RANDOM_FOREST:

            # Monte Carlo feature selection for Random Forest
            tree.monte_carlo_random_treefeature_selection(
                feature_table,
                labels,
                dataset_dir,
                n_iterations=100,
                test_size=test_size
            )

            feature_table, feature_array, _, labels = feature_tools.load_feature_table(
                dataset_dir, "feature_random_forest_monte_carlo.csv"
            )
            model_rf, metrics_rf = tree.train_random_forest(
                feature_array,
                labels,
                test_size=test_size,
                n_estimators=100
            )
            print("Trained Random Forest model:", model_rf)
            print("Metrics (Random Forest):", metrics_rf)

            # Plot random forest tree
            plot_random_forest(feature_table, model_rf)

            viz_tools.plot_regression(
                labels, model_rf.predict(feature_array).flatten()
            )

        else:

            # Monte Carlo feature selection for Gradient Boosted Tree
            tree.monte_carlo_gradient_boosted_tree_feature_selection(
                feature_table,
                labels,
                dataset_dir,
                n_iterations=100,
                test_size=test_size
            )

            feature_table, feature_array, _, labels = feature_tools.load_feature_table(
                dataset_dir, "feature_gradient_boosted_tree_monte_carlo.csv"
            )
            model, metrics = tree.train_gradient_boosted_tree(
                feature_array,
                labels,
                test_size=test_size,
                n_estimators=100
            )
            print("Trained Gradient Boosted Tree model:", model)
            print("Metrics (Gradient Boosted Tree):", metrics)

            viz_tools.plot_regression(
                labels, model.predict(feature_array).flatten()
            )
        
        plt.show()