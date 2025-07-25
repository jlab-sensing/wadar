# This script trains various regression machine learning models on a dataset with handcrafted features.

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
from pickle import dump
from _04_athena.multi_later_percepetron import MultiLaterPercepetron, monte_carlo_mlp_feature_selection

# Enable and disable different model training and evaluation sections as needed.
REGRESSION = False
RANDOM_FOREST = False
GRADIENT_BOOSTED_TREE = False
SVR = False
NEURAL_NETWORKS = True

def save_sklearn_model(dataset_dir, poly_model, model_name):
    save_path = os.path.join(dataset_dir, f"models/model_{model_name.lower().replace(' ', '_')}.pkl")
    with open(save_path, 'wb') as f:
        dump(poly_model, f)

if __name__ == "__main__":

    n_iterations = 100          # Number of iterations for Monte Carlo feature selection
    
    # When combining datasets,
    # dataset_dirs = [
    #     "../data/wet-0-soil-compaction-dataset",
    #     "../data/wet-1-soil-compaction-dataset",
    #     "../data/wet-2-soil-compaction-dataset"]
    # target_dir = "../data/training-dataset"
    # dataset_raw = dataset.combine_datasets(dataset_dirs, target_dir)
    # dataset_dir = target_dir
    # new_dataset = True

    # When the dataset is already combined,
    dataset_dir = "../data/training-dataset"
    new_dataset = False

    hydros = loader.FrameLoader(dataset_dir, new_dataset=new_dataset, ddc_flag=True)
    X, y = hydros.X, hydros.y

    if new_dataset:
        hephaestus_features = feature_tools.FeatureTools(X)
        feature_table = hephaestus_features.feature_full_monty(y, dataset_dir)

    feature_table, feature_array, feature_names, labels = feature_tools.load_feature_table(dataset_dir)

    # ==============
    # Neural networks
    # ==============

    if NEURAL_NETWORKS:

        # Crashing for some reason
        # monte_carlo_mlp_feature_selection(feature_table, labels, dataset_dir, n_iterations=n_iterations)

        # _, feature_array, feature_names, labels = feature_tools.load_feature_table(
        #     dataset_dir, "models/feature_mlp_monte_carlo.csv"
        # )

        _, feature_array, feature_names, labels = feature_tools.load_feature_table(dataset_dir)


        mlp = MultiLaterPercepetron(feature_array, labels)
        model, metrics = mlp.full_monty()

        mae = metrics["mae"]
        rmse = metrics["rmse"]
        r2 = metrics["r2"]
        training_time = metrics["training_time"]
        inference_time = metrics["inference_time"]
        accuracy = metrics["accuracy"]

        update_results("Handcrafted", "MLP", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
        mlp.save_model(model, dataset_dir)

    # ==============
    # Evaluating ridge regression models
    # ==============

    if REGRESSION:

        print("Evaluating ridge regression models...")

        for degree in [1, 2, 3]:

            print(f"Degree {degree} Monte Carlo Feature Selection")

            mc_results = regression.monte_carlo_regression_feature_selection(
                feature_table, labels, dataset_dir, degree=degree, n_iterations=n_iterations
            )

            feature_table, feature_array, _, labels = feature_tools.load_feature_table(
                dataset_dir, f"models/feature_linear_regression_{degree}_monte_carlo.csv"
            )
            poly_model, poly_metrics = regression.polynomial_regression(
                feature_array, labels, degree=degree, kfold_splits=5
            )

        for degree in [1, 2, 3]:

            _, feature_array, _, labels = feature_tools.load_feature_table(
                dataset_dir, f"models/feature_linear_regression_{degree}_monte_carlo.csv"
            )

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

            update_results("Handcrafted", model_name, accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
            save_sklearn_model(dataset_dir, poly_model, model_name)

        print()
        print()

    # ==============
    # Evaluating Random Forest models
    # ==============

    if RANDOM_FOREST:

        print("Evaluating Random Forest models...")

        tree.monte_carlo_random_tree_feature_selection(
            feature_table,
            labels,
            dataset_dir,
            n_iterations=n_iterations,
        )

        feature_table, feature_array, _, labels = feature_tools.load_feature_table(
            dataset_dir, "models/feature_random_forest_monte_carlo.csv"
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

        update_results("Handcrafted", "Random Forest", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
        save_sklearn_model(dataset_dir, model_rf, "Random Forest")

        print()
        print()

    # ==============
    # Evaluating Gradient Boosted Tree models
    # ==============

    if GRADIENT_BOOSTED_TREE:

        print("Evaluating Gradient Boosted Tree models...")

        tree.monte_carlo_gradient_boosted_tree_feature_selection(
            feature_table,
            labels,
            dataset_dir,
            n_iterations=n_iterations,
        )

        feature_table, feature_array, _, labels = feature_tools.load_feature_table(
            dataset_dir, "models/feature_gradient_boosted_tree_monte_carlo.csv"
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

        update_results("Handcrafted", "Gradient Boosted Tree", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
        save_sklearn_model(dataset_dir, model, "Gradient Boosted Tree")

        print()
        print()

    # ==============
    # Evaluating SVR models
    # ==============

    if SVR:

        print("Evaluating SVR models...")

        feature_table = svr.monte_carlo_svr_feature_selection(
            feature_table, labels, dataset_dir, n_iterations=n_iterations
        )
        
        feature_table, feature_array, feature_names, labels = feature_tools.load_feature_table(
            dataset_dir, "models/feature_svr_monte_carlo.csv"
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

        update_results("Handcrafted", "SVR", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
        save_sklearn_model(dataset_dir, model, "SVR")