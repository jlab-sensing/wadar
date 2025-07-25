# This script trains various regression machine learning models on a dataset with learned features.

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
from _05_apollo.viz_tools import plot_accuracy_mae
from pickle import dump, load
from _06_hermes.parameters import model_name, feature_extraction_name

IMPROVEMENT_THRESHOLD = 1e-4  # threshold for improvement in MAE to consider
N_COMPONENTS = 16  



def save_sklearn_model(dataset_dir, poly_model, name):
    save_path = os.path.join(dataset_dir, f"{name.lower().replace(' ', '_')}")
    with open(save_path, 'wb') as f:
        dump(poly_model, f)

def evaluate_classical_models(features_magnitude, y, feature_name, dataset_dir, model_dir):

    # Use all features for evaluation
    print(f"Evaluating models for {feature_name} using all features...")

    # ==============
    # Regression models
    # ==============
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
        save_sklearn_model(model_dir, model, model_name(feature_name, f"Regression Degree {degree}"))

    # ==============
    # Random Forest
    # ==============
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
    save_sklearn_model(model_dir, model, model_name(feature_name, "Random Forest"))

    # ==============
    # Gradient Boosted Tree
    # ==============
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
    save_sklearn_model(model_dir, model, model_name(feature_name, "Gradient Boosted Tree"))

    # ==============
    # Support Vector Regression
    # ==============
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
    save_sklearn_model(model_dir, model, model_name(feature_name, "SVR"))

if __name__ == "__main__":

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

    # ==============================================================================

    hydros = loader.FrameLoader(dataset_dir, new_dataset=new_dataset, ddc_flag=True)
    X, y = hydros.X, hydros.y

    model_dir = os.path.join(dataset_dir, "models")
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)

    # ==============
    # For the PCA Amplitude
    # ==============

    X = np.abs(X)
    hephaestus = PCAProcessor(X, n_components=N_COMPONENTS)
    features_amplitude = hephaestus.dimensionality_reduction()

    save_sklearn_model(model_dir, hephaestus, feature_extraction_name("PCA Amplitude"))

    evaluate_classical_models(features_amplitude, y, "PCA Amplitude", dataset_dir, model_dir)

    # ==============
    # For the PCA Angle
    # ==============

    X = np.angle(X)
    hephaestus = PCAProcessor(X, n_components=N_COMPONENTS)
    features_phase = hephaestus.dimensionality_reduction()

    save_sklearn_model(model_dir, hephaestus, feature_extraction_name("PCA Phase"))

    evaluate_classical_models(features_phase, y, "PCA Phase", dataset_dir, model_dir)

    # ==============
    # For the PCA Combined
    # ==============

    num_of_features = int(N_COMPONENTS / 2)
    features = np.concatenate([features_phase[:, :num_of_features], features_amplitude[:, :num_of_features]], axis=1)

    evaluate_classical_models(features, y, "PCA Combined", dataset_dir, model_dir)

    results = pd.read_csv(os.path.join(dataset_dir, "learned_classical_results.csv"))

    plot_accuracy_mae(results)