import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _06_hermes.bulk_density_labels import bulk_density_to_label
from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools
import _04_athena.sgd as sgd
import pandas as pd
from _05_apollo import viz_tools


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

    # ========== Regression Example ==========

    print("SGD Regression")

    print("Optimizing hyperparameters with grid search")

    clf = sgd.grid_search_sgd_regression(feature_array, labels, test_size=test_size, random_state=21)

    best_params = clf.best_params_
    
    print(f"Best parameters: {best_params}")

    print("Training model with best parameters")

    model, metrics = sgd.sgd_regression(feature_array, labels, test_size=test_size, random_state=2,
                                            eta0=best_params['sgdregressor__eta0'],
                                            max_iter=best_params['sgdregressor__max_iter'],
                                            tol=best_params['sgdregressor__tol'])

    print(f"Model metrics: {metrics}")
    print()

    # ========== Classification Example ==========
    
    print("SGD Classification")

    print("Optimizing hyperparameters with grid search")

    clf = sgd.grid_search_sgd_classification(feature_array, labels, test_size=test_size, random_state=2)

    best_params = clf.best_params_

    print(f"Best parameters: {best_params}")

    print("Training model with best parameters")

    model, metrics = sgd.sgd_classification(feature_array, labels, test_size=test_size, random_state=2,
                                            eta0=best_params['sgdclassifier__eta0'],
                                            max_iter=best_params['sgdclassifier__max_iter'],
                                            tol=best_params['sgdclassifier__tol'])
    print(f"Model metrics: {metrics}")

    # ========== Confusion Matrix ==========

    y_labels = []
    for i, label in enumerate(labels):
        y_labels.append(bulk_density_to_label(label))

    y_pred = model.predict(feature_array)

    viz_tools.plot_confusion_matrix(y_labels, y_pred)
    plt.show()