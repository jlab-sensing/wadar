# This script evaluates various classical machine learning models on a dataset with learned features
# from classical methods.

import os
import sys
import numpy as np

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
from _03_hephaestus.pca_tools import PCAProcessor
import pickle

IMPROVEMENT_THRESHOLD = 1e-4  # threshold for improvement in MAE to consider
N_COMPONENTS = 16  

def evaluate_models(features_magnitude, y, feature_name, dataset_dir):

    # Use all features for evaluation
    print(f"Evaluating regression models for {feature_name} using all features...")

    # Regression models
    for degree in [1, 2, 3]:
        try:
            model, metrics = regression.polynomial_regression(features_magnitude, y, degree=degree)
        except Exception:
            continue

        update_results(
            feature_name,
            f"Regression Degree {degree}",
            metrics['accuracy'],
            metrics['mae'],
            metrics['rmse'],
            metrics['r2'],
            metrics['training_time'],
            metrics['inference_time'],
            dataset_dir,
            results_file="learned_classical_results.csv"
        )
        print(f"Regression Degree {degree} with all features achieved MAE: {metrics['mae']:.4f}")

    # Random Forest
    print(f"\nEvaluating Random Forest models for {feature_name} using all features...")
    model, metrics = tree.train_random_forest(features_magnitude, y, n_estimators=100)
    update_results(
        feature_name,
        f"Random Forest",
        metrics['accuracy'],
        metrics['mae'],
        metrics['rmse'],
        metrics['r2'],
        metrics['training_time'],
        metrics['inference_time'],
        dataset_dir,
        results_file="learned_classical_results.csv"
    )
    print(f"Random Forest with all features achieved MAE: {metrics['mae']:.4f}")

    # Gradient Boosted Tree
    print(f"\nEvaluating Gradient Boosted Tree models for {feature_name} using all features...")
    model, metrics = tree.train_gradient_boosted_tree(features_magnitude, y, n_estimators=100)
    update_results(
        feature_name,
        f"Gradient Boosted Tree",
        metrics['accuracy'],
        metrics['mae'],
        metrics['rmse'],
        metrics['r2'],
        metrics['training_time'],
        metrics['inference_time'],
        dataset_dir,
        results_file="learned_classical_results.csv"
    )
    print(f"Gradient Boosted Tree with all features achieved MAE: {metrics['mae']:.4f}")

    # Support Vector Regression
    print(f"\nEvaluating Support Vector Regression models for {feature_name} using all features...")
    best_params = svr.tune_svr(features_magnitude, y)
    model, metrics = svr.svr_regression(
        features_magnitude,
        y,
        C=best_params['C'],
        gamma=best_params['gamma'],
        epsilon=best_params['epsilon']
    )
    update_results(
        feature_name,
        f"SVR",
        metrics['accuracy'],
        metrics['mae'],
        metrics['rmse'],
        metrics['r2'],
        metrics['training_time'],
        metrics['inference_time'],
        dataset_dir,
        results_file="learned_classical_results.csv"
    )
    print(f"SVR with all features achieved MAE: {metrics['mae']:.4f}")

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

    model_dir = os.path.join(dataset_dir, "models")
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)

    X = np.abs(X)
    hephaestus = PCAProcessor(X, n_components=N_COMPONENTS)
    features_magnitude = hephaestus.dimensionality_reduction()

    evaluate_models(features_magnitude, y, "PCA Magnitude", dataset_dir)

    X = np.angle(X)
    hephaestus = PCAProcessor(X, n_components=N_COMPONENTS)
    features_angle = hephaestus.dimensionality_reduction()

    evaluate_models(features_angle, y, "PCA Angle", dataset_dir)

    num_of_features = int(N_COMPONENTS / 2)
    features = np.concatenate([features_angle[:, :num_of_features], features_magnitude[:, :num_of_features]], axis=1)

    evaluate_models(features, y, "PCA Combined", dataset_dir)