import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.dataset import bulk_density_to_label
from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools
import _04_athena.sgd as sgd
import pandas as pd
from _05_apollo import viz_tools


if __name__ == "__main__":

    dataset_dir = "../data/dry-soil-compaction-dataset"
    feature_file_name = "features_selected.csv"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    # features = feature_tools.lasso_minimize_features(dataset_dir, X, y) # should already be done

    # ========== Regression Example ==========

    print("SGD Regression")

    print("Optimizing hyperparameters with grid search")

    clf = sgd.grid_search_sgd_regression(dataset_dir, feature_file_name, test_size=0.2, random_state=2)

    best_params = clf.best_params_
    
    print(f"Best parameters: {best_params}")

    print("Training model with best parameters")

    model, metrics = sgd.sgd_regression(dataset_dir, feature_file_name, test_size=0.2, random_state=2,
                                            eta0=best_params['sgdregressor__eta0'],
                                            max_iter=best_params['sgdregressor__max_iter'],
                                            tol=best_params['sgdregressor__tol'])

    print(f"Model metrics: {metrics}")
    print()

    # ========== Classification Example ==========
    
    print("SGD Classification")

    print("Optimizing hyperparameters with grid search")

    clf = sgd.grid_search_sgd_classification(dataset_dir, feature_file_name, test_size=0.2, random_state=2)

    best_params = clf.best_params_

    print(f"Best parameters: {best_params}")

    print("Training model with best parameters")

    model, metrics = sgd.sgd_classification(dataset_dir, feature_file_name, test_size=0.2, random_state=2,
                                            eta0=best_params['sgdclassifier__eta0'],
                                            max_iter=best_params['sgdclassifier__max_iter'],
                                            tol=best_params['sgdclassifier__tol'])
    print(f"Model metrics: {metrics}")

    # ========== Confusion Matrix ==========

    data = pd.read_csv(dataset_dir + '/' + feature_file_name)
    X = data.drop(columns=['label']).values
    y = data['label'].values

    y_labels = []
    for i, label in enumerate(y):
        y_labels.append(bulk_density_to_label(label))

    y_pred = model.predict(X)

    viz_tools.plot_confusion_matrix(y_labels, y_pred)