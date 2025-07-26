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
from _03_hephaestus.autoencoder import AutoencoderFeatureSelector
from _04_athena.pretrained_cnn import PretrainedCNNFeatureExtractor
from _04_athena.multi_later_percepetron import MultiLaterPercepetron, monte_carlo_mlp_feature_selection

IMPROVEMENT_THRESHOLD = 1e-4  # threshold for improvement in MAE to consider
N_COMPONENTS = 16  

PCA = True
AUTOENCODER = True
PRETRAINED_CNN = True


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

    # ==============
    # Multi-Layer Perceptron
    # ==============

    mlp = MultiLaterPercepetron(features_magnitude, y)
    model, metrics = mlp.full_monty()
    update_results(
        feature_name,
        f"Multi-Layer Perceptron",
        metrics['accuracy'],
        metrics['mae'],
        metrics['rmse'],
        metrics['r2'],
        metrics['training_time'],
        metrics['inference_time'],
        dataset_dir,
        results_file="learned_classical_results.csv"
    )

    file_name = model_name(feature_name, "MLP")[:-4]
    mlp.save_model(model, model_dir, name=file_name)

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

    image_dir = os.path.join(dataset_dir, "images")

    # ==============================================================================

    hydros = loader.FrameLoader(dataset_dir, new_dataset=new_dataset, ddc_flag=True)
    X, y = hydros.X, hydros.y

    model_dir = os.path.join(dataset_dir, "models")
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)

    if PCA:

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

    if AUTOENCODER:
        # ==============
        # Autoencoder
        # ==============

        epochs = 30

        feature_extractor = AutoencoderFeatureSelector(X, signal_type='magnitude', encoding_dim=N_COMPONENTS)
        features_autoencoder = feature_extractor.fit(epochs=epochs, batch_size=32, test_size=0.2)
        feature_extractor.save_model(model_dir, "feature_autoencoder_amplitude.keras")

        evaluate_classical_models(features_autoencoder, y, "Autoencoder Amplitude", dataset_dir, model_dir)

        feature_extractor = AutoencoderFeatureSelector(X, signal_type='phase', encoding_dim=N_COMPONENTS)
        features_autoencoder_phase = feature_extractor.fit(epochs=epochs, batch_size=32, test_size=0.2)
        feature_extractor.save_model(model_dir, "feature_autoencoder_phase.keras")
        evaluate_classical_models(features_autoencoder_phase, y, "Autoencoder Phase", dataset_dir, model_dir)

        # Combine the features from both autoencoders
        num_of_features = int(N_COMPONENTS / 2)
        features_autoencoder_combined = np.concatenate(
            [features_autoencoder_phase[:, :num_of_features], features_autoencoder[:, :num_of_features]], axis=1
        )
        evaluate_classical_models(features_autoencoder_combined, y, "Autoencoder Combined", dataset_dir, model_dir)

    if PRETRAINED_CNN:
        # ==============
        # Pretrained CNN
        # ==============

        epochs = 30

        X = np.abs(X)
        feature_extractor = PretrainedCNNFeatureExtractor(X,
                                                        output_dir=model_dir,
                                                        dimensions=N_COMPONENTS)
        model, features_cnn = feature_extractor.full_monty(epochs=epochs)

        feature_extractor.save_model(model_dir + "/feature_cnn_amplitude.keras")

        evaluate_classical_models(features_cnn, y, "CNN Amplitude", dataset_dir, model_dir)

        X = np.angle(X)
        feature_extractor = PretrainedCNNFeatureExtractor(X,
                                                        output_dir=model_dir,
                                                        dimensions=N_COMPONENTS)
        model, features_cnn_phase = feature_extractor.full_monty(epochs=epochs)

        feature_extractor.save_model(model_dir + "/feature_cnn_phase.keras")
        evaluate_classical_models(features_cnn_phase, y, "CNN Phase", dataset_dir, model_dir)

        # Combine the features from both CNNs
        num_of_features = int(N_COMPONENTS / 2)
        features_cnn_combined = np.concatenate(
            [features_cnn_phase[:, :num_of_features], features_cnn[:, :num_of_features]], axis=1
        )
        evaluate_classical_models(features_cnn_combined, y, "CNN Combined", dataset_dir, model_dir)

    # ======================================================================

    results = pd.read_csv(os.path.join(dataset_dir, "learned_classical_results.csv"))
    plot_accuracy_mae(results)