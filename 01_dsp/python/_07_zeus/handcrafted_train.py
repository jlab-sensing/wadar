# This script trains various regression machine learning models on a dataset with handcrafted features.

import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
sys.path.insert(0, parent_dir)
from _01_gaia import loader
from _03_hephaestus import feature_tools
from _04_athena import regression
from _06_hermes.logger import update_results
from _04_athena import tree
import _04_athena.svr as svr
from pickle import dump
from _04_athena.multi_later_percepetron import MultiLaterPercepetron

# Enable and disable different model training sections as needed.
REGRESSION = True
RANDOM_FOREST = True
GRADIENT_BOOSTED_TREE = True
SVR = True
NEURAL_NETWORKS = True

def save_sklearn_model(dataset_dir, poly_model, model_name):
    save_path = os.path.join(dataset_dir, f"models/model_{model_name.lower().replace(' ', '_')}.pkl")
    with open(save_path, 'wb') as f:
        dump(poly_model, f)

if __name__ == "__main__":

    n_iterations = 100  # Number of iterations for Monte Carlo feature selection
    
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
        print("Training Multi-Layer Perceptron model...")

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
        
        print(f"MLP Training Complete - Accuracy: {accuracy:.3f}, MAE: {mae:.3f}")
        print()

    # ==============
    # Ridge Regression Models
    # ==============

    if REGRESSION:
        print("Training ridge regression models...")

        for degree in [1, 2, 3]:
            print(f"Training Degree {degree} regression with Monte Carlo feature selection...")

            # Perform Monte Carlo feature selection
            regression.monte_carlo_regression_feature_selection(
                feature_table, labels, dataset_dir, degree=degree, n_iterations=n_iterations
            )

            # Load selected features and train model
            _, feature_array, _, labels = feature_tools.load_feature_table(
                dataset_dir, f"models/feature_linear_regression_{degree}_monte_carlo.csv"
            )

            poly_model, poly_metrics = regression.polynomial_regression(
                feature_array, labels, degree=degree, kfold_splits=5
            )

            # Extract metrics
            mae = poly_metrics["mae"]
            rmse = poly_metrics["rmse"]
            r2 = poly_metrics["r2"]
            accuracy = poly_metrics["accuracy"]
            inference_time = poly_metrics["inference_time"]
            training_time = poly_metrics["training_time"]

            model_name = f"Regression Degree {degree}"

            # Save results and model
            update_results("Handcrafted", model_name, accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
            save_sklearn_model(dataset_dir, poly_model, model_name)
            
            print(f"Degree {degree} Complete - Accuracy: {accuracy:.3f}, MAE: {mae:.3f}")

        print()

    # ==============
    # Random Forest Models
    # ==============

    if RANDOM_FOREST:
        print("Training Random Forest models...")

        # Perform Monte Carlo feature selection
        tree.monte_carlo_random_tree_feature_selection(
            feature_table, labels, dataset_dir, n_iterations=n_iterations
        )

        # Load selected features and train model
        feature_table, feature_array, _, labels = feature_tools.load_feature_table(
            dataset_dir, "models/feature_random_forest_monte_carlo.csv"
        )

        model_rf, metrics_rf = tree.train_random_forest(feature_array, labels)

        # Extract metrics
        mae = metrics_rf["mae"]
        r2 = metrics_rf["r2"]
        accuracy = metrics_rf["accuracy"]
        inference_time = metrics_rf["inference_time"]
        training_time = metrics_rf["training_time"]
        rmse = metrics_rf["rmse"]

        # Save results and model
        update_results("Handcrafted", "Random Forest", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
        save_sklearn_model(dataset_dir, model_rf, "Random Forest")
        
        print(f"Random Forest Complete - Accuracy: {accuracy:.3f}, MAE: {mae:.3f}")
        print()

    # ==============
    # Gradient Boosted Tree Models
    # ==============

    if GRADIENT_BOOSTED_TREE:
        print("Training Gradient Boosted Tree models...")

        # Perform Monte Carlo feature selection
        tree.monte_carlo_gradient_boosted_tree_feature_selection(
            feature_table, labels, dataset_dir, n_iterations=n_iterations
        )

        # Load selected features and train model
        feature_table, feature_array, _, labels = feature_tools.load_feature_table(
            dataset_dir, "models/feature_gradient_boosted_tree_monte_carlo.csv"
        )

        model, metrics = tree.train_gradient_boosted_tree(feature_array, labels)

        # Extract metrics
        mae = metrics["mae"]
        r2 = metrics["r2"]
        accuracy = metrics["accuracy"]
        inference_time = metrics["inference_time"] 
        training_time = metrics["training_time"]
        rmse = metrics["rmse"]

        # Save results and model
        update_results("Handcrafted", "Gradient Boosted Tree", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
        save_sklearn_model(dataset_dir, model, "Gradient Boosted Tree")
        
        print(f"Gradient Boosted Tree Complete - Accuracy: {accuracy:.3f}, MAE: {mae:.3f}")
        print()

    # ==============
    # Support Vector Regression Models
    # ==============

    if SVR:
        print("Training SVR models...")

        # Perform Monte Carlo feature selection
        svr.monte_carlo_svr_feature_selection(
            feature_table, labels, dataset_dir, n_iterations=n_iterations
        )
        
        # Load selected features and train model
        feature_table, feature_array, feature_names, labels = feature_tools.load_feature_table(
            dataset_dir, "models/feature_svr_monte_carlo.csv"
        )

        model, metrics = svr.svr_regression(feature_array, labels)

        # Extract metrics
        mae = metrics['mae']
        r2 = metrics['r2']
        accuracy = metrics['accuracy']
        inference_time = metrics['inference_time']
        training_time = metrics['training_time']
        rmse = metrics['rmse']

        # Save results and model
        update_results("Handcrafted", "SVR", accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir)
        save_sklearn_model(dataset_dir, model, "SVR")
        
        print(f"SVR Complete - Accuracy: {accuracy:.3f}, MAE: {mae:.3f}")

    print("Training complete!")