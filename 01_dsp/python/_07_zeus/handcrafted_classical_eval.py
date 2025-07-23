# This script evaluates various classical machine learning models on a dataset with handcrafted features.

import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia import dataset
from _01_gaia import loader
from _03_hephaestus import feature_tools
from _04_athena import regression
from _06_hermes.logger import update_results
import pandas as pd
from _04_athena import tree
import _04_athena.svr as svr

if __name__ == "__main__":

    n_iterations = 1000 # maybe this should be a parameter?

    dataset_dirs = []

    args = sys.argv[1:]

    if len(args) < 1:
        print("Usage: python PrepareDataset.py <dataset_dir> [new_dataset]")
        sys.exit(1)

    if len(args) == 2:
        dataset_dir = args[0]
        dataset_raw = dataset.Dataset(dataset_dir)
    else:
        target_dir = args[0]
        dataset_dirs = args[1:]
        dataset_raw = dataset.combine_datasets(dataset_dirs, target_dir)
        dataset_dir = target_dir

    if os.path.exists(dataset_dir + "/X_raw.npy") and os.path.exists(dataset_dir + "/y_raw.npy"):
        new_dataset = False
    else:
        new_dataset = True

    hydros = loader.FrameLoader(dataset_dir, new_dataset=new_dataset, ddc_flag=True)
    X, y = hydros.X, hydros.y

    print("X shape:", X.shape)
    print("y shape:", y.shape)

    hephaestus_features = feature_tools.FeatureTools(X)
    feature_table = hephaestus_features.feature_full_monty(y, dataset_dir)

    feature_table, feature_array, feature_names, labels = feature_tools.load_feature_table(dataset_dir)

    print("Feature table shape:", feature_table.shape)


    # ==============
    # Evaluating ridge regression models
    # ==============

    print("Evaluating ridge regression models...")

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

    for degree in [1, 2, 3]:

        _, feature_array, _, labels = feature_tools.load_feature_table(
            dataset_dir, f"feature_linear_regression_{degree}_monte_carlo.csv")

        poly_model, poly_metrics = regression.polynomial_regression(
            feature_array, labels, degree=degree, kfold_splits=5
        )

        mae = poly_metrics["mae"]
        rmse = poly_metrics["rmse"]
        r2 = poly_metrics["r2"]
        accuracy = poly_metrics["accuracy"]
        inference_time = poly_metrics["inference_time"]
        training_time = poly_metrics["training_time"]

        model_name = f"Regression Degree {degree}"

        update_results(model_name, accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
    
    print()
    print()

    # ==============
    # Evaluating Random Forest models
    # ==============

    print("Evaluating Random Forest models...")

    tree.monte_carlo_random_tree_feature_selection(
        feature_table,
        labels,
        dataset_dir,
        n_iterations=n_iterations,
    )

    feature_table, feature_array, _, labels = feature_tools.load_feature_table(
        dataset_dir, "feature_random_forest_monte_carlo.csv"
    )

    model_rf, metrics_rf = tree.train_random_forest(
        feature_array,
        labels
    )

    mae = metrics_rf["mae"]
    r2 = metrics_rf["r2"]
    accuracy = metrics_rf["accuracy"]
    inference_time = metrics_rf["inference_time"]
    training_time = metrics_rf["training_time"]
    rmse = metrics_rf["rmse"]

    update_results("Random Forest", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)

    print()
    print()

    # ==============
    # Evaluating Gradient Boosted Tree models
    # ==============

    print("Evaluating Gradient Boosted Tree models...")

    tree.monte_carlo_gradient_boosted_tree_feature_selection(
        feature_table,
        labels,
        dataset_dir,
        n_iterations=n_iterations,
    )

    feature_table, feature_array, _, labels = feature_tools.load_feature_table(
        dataset_dir, "feature_gradient_boosted_tree_monte_carlo.csv"
    )

    model, metrics = tree.train_gradient_boosted_tree(
        feature_array,
        labels
    )

    mae = metrics["mae"]
    r2 = metrics["r2"]
    accuracy = metrics["accuracy"]
    inference_time = metrics["inference_time"] 
    training_time = metrics["training_time"]
    rmse = metrics["rmse"]

    update_results("Gradient Boosted Tree", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)

    print()
    print()

    # ==============
    # Evaluating SVR models
    # ==============

    print("Evaluating SVR models...")

    feature_table = svr.monte_carlo_svr_feature_selection(
        feature_table, labels, dataset_dir, n_iterations=n_iterations
    )
    
    feature_table, feature_array, feature_names, labels = feature_tools.load_feature_table(
        dataset_dir, "feature_svr_monte_carlo.csv"
    )

    model, metrics = svr.svr_regression(
        feature_array, labels
    )

    mae = metrics['mae']
    r2 = metrics['r2']
    accuracy = metrics['accuracy']
    inference_time = metrics['inference_time']
    training_time = metrics['training_time']
    rmse = metrics['rmse']

    update_results("SVR", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)