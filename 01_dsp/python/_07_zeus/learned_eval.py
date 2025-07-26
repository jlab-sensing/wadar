# This script evaluates various regression machine learning models on a dataset with learned features.

import os
import sys
import numpy as np

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _06_hermes.logger import update_results
import pandas as pd
from pickle import dump, load
from sklearn.metrics import mean_absolute_error, r2_score
import numpy as np
from _06_hermes.parameters import num2label
from _05_apollo.viz_tools import plot_accuracy_mae
from _06_hermes.parameters import model_name, feature_extraction_name
from _01_gaia import loader
from learned_train import N_COMPONENTS
from _03_hephaestus.autoencoder import AutoencoderFeatureSelector
from _04_athena.pretrained_cnn import PretrainedCNNFeatureExtractor

N_COMPONENTS = 16  

PCA = True
AUTOENCODER = True
PRETRAINED_CNN = True

def load_sklearn_model(models_dir, model_name):
    model_path = os.path.join(models_dir, model_name)
    with open(model_path, 'rb') as f:
        poly_model = load(f)
    return poly_model

def evaluate_model(dataset_dir, y, predictions, model_name, features_name):
    mae = mean_absolute_error(y, predictions)
    rmse = np.sqrt(np.mean((y - predictions) ** 2))
    r2 = r2_score(y, predictions)

    y_labels = [num2label(label) for label in y]
    predictions_labels = [num2label(pred) for pred in predictions]
    accuracy = np.mean(np.array(y_labels) == np.array(predictions_labels))

    update_results(
            features_name,
            model_name,
            accuracy,
            mae,
            rmse,
            r2,
            0,  # Training time not applicable here
            0,  # Inference time not applicable here
            dataset_dir,
            f"results_learned_farm.csv"
        )

if __name__ == "__main__":

    # Validation dataset
    dataset_dir = "../data/training-dataset"
    new_dataset = False

    # ==============================================================================

    hydros = loader.FrameLoader(dataset_dir, new_dataset=new_dataset, ddc_flag=True)
    X, y = hydros.X, hydros.y

    models = ["Regression Degree 1", "Regression Degree 2", "Regression Degree 3",
                    "Random Forest", "Gradient Boosted Tree", "SVR"]
    model_dir = os.path.join(dataset_dir, "models")

    if PCA:

        feature_selection = ["PCA Amplitude", "PCA Phase"]

        # PCA Amplitude

        pca_amplitude_extractor = load_sklearn_model(model_dir, feature_extraction_name("PCA Amplitude"))
        features_amplitude = pca_amplitude_extractor.dimensionality_reduction(np.abs(X))

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("PCA Amplitude", model_type))
            predictions = regression_model.predict(features_amplitude)
            evaluate_model(dataset_dir, y, predictions, model_type, "Amplitude")

        # PCA Phase
        pca_phase_extractor = load_sklearn_model(model_dir, feature_extraction_name("PCA Phase"))
        features_phase = pca_phase_extractor.dimensionality_reduction(np.angle(X))

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("PCA Phase", model_type))
            predictions = regression_model.predict(features_phase)
            evaluate_model(dataset_dir, y, predictions, model_type, "Phase")

        # Combined features from both PCA
        num_of_features = int(N_COMPONENTS / 2)
        features_combined = np.concatenate(
            [features_phase[:, :num_of_features], features_amplitude[:, :num_of_features]], axis=1
        )

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("PCA Combined", model_type))
            predictions = regression_model.predict(features_combined)
            evaluate_model(dataset_dir, y, predictions, model_type, "Combined")

    if AUTOENCODER:
        feature_selection = ["Autoencoder Amplitude", "Autoencoder Phase"]

        # Autoencoder Amplitude
        autoencoder_amplitude_extractor = AutoencoderFeatureSelector(
            X, encoding_dim=N_COMPONENTS
        )
        autoencoder_amplitude_extractor.load_model(model_dir, "feature_autoencoder_amplitude.keras")
        X_flat = autoencoder_amplitude_extractor.pre_process(X, signal_type='magnitude')
        features_amplitude = autoencoder_amplitude_extractor.transform(X_flat)

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("Autoencoder Amplitude", model_type))
            predictions = regression_model.predict(features_amplitude)
            evaluate_model(dataset_dir, y, predictions, model_type, "Autoencoder Amplitude")

        # Autoencoder Phase
        autoencoder_phase_extractor = AutoencoderFeatureSelector(
            X=np.angle(X), encoding_dim=N_COMPONENTS
        )
        autoencoder_phase_extractor.load_model(model_dir, "feature_autoencoder_phase.keras")
        X_flat = autoencoder_phase_extractor.pre_process(X, signal_type='phase')
        features_phase = autoencoder_phase_extractor.transform(X_flat)

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("Autoencoder Phase", model_type))
            predictions = regression_model.predict(features_phase)
            evaluate_model(dataset_dir, y, predictions, model_type, "Autoencoder Phase")

        # Combined features from both Autoencoders
        num_of_features = int(N_COMPONENTS / 2)
        features_combined = np.concatenate(
            [features_phase[:, :num_of_features], features_amplitude[:, :num_of_features]], axis=1
        )

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("Autoencoder Combined", model_type))
            predictions = regression_model.predict(features_combined)
            evaluate_model(dataset_dir, y, predictions, model_type, "Autoencoder Combined")

    if PRETRAINED_CNN:

        feature_selection = ["CNN Amplitude", "CNN Phase"]
        feature_names = ["feature_cnn_amplitude.keras", "feature_cnn_phase.keras"]

        # CNN Amplitude

        trainer = PretrainedCNNFeatureExtractor(X, output_dir=dataset_dir + "/images", dimensions=N_COMPONENTS)
        trainer.load_model(model_dir + "/" + "feature_cnn_amplitude.keras")
        features = trainer.predict(np.abs(X))

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("CNN Amplitude", model_type))
            predictions = regression_model.predict(features)
            evaluate_model(dataset_dir, y, predictions, model_type, "CNN Amplitude")

        # CNN Phase

        trainer = PretrainedCNNFeatureExtractor(X, output_dir=dataset_dir + "/images", dimensions=N_COMPONENTS)
        trainer.load_model(model_dir + "/" + "feature_cnn_phase.keras")
        features_phase = trainer.predict(np.angle(X))

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("CNN Phase", model_type))
            predictions = regression_model.predict(features_phase)
            evaluate_model(dataset_dir, y, predictions, model_type, "CNN Phase")

        # Combined features from both CNNs

        num_of_features = int(N_COMPONENTS / 2)
        features_combined = np.concatenate(
            [features_phase[:, :num_of_features], features[:, :num_of_features]], axis=1
        )

        for model_type in models:
            regression_model = load_sklearn_model(model_dir, model_name("CNN Combined", model_type))
            predictions = regression_model.predict(features_combined)
            evaluate_model(dataset_dir, y, predictions, model_type, "CNN Combined")
    
    # ==============

    results = pd.read_csv(os.path.join(dataset_dir, "results_learned_farm.csv"))
    plot_accuracy_mae(results)