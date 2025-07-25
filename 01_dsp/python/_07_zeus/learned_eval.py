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

    feature_selection = ["PCA Amplitude", "PCA Phase"]
    models = ["Regression Degree 1", "Regression Degree 2", "Regression Degree 3",
              "Random Forest", "Gradient Boosted Tree", "SVR"]
    
    for feature_name in feature_selection:
        for model_type in models:

            model_name_full = model_name(feature_name, model_type)
            model_dir = os.path.join(dataset_dir, "models")
            poly_model = load_sklearn_model(model_dir, model_name_full)

            print(f"Evaluating {model_type} with {feature_name}...")

            pca_feature_extractor = load_sklearn_model(model_dir, feature_extraction_name(feature_name))
            features = pca_feature_extractor.dimensionality_reduction()

            predictions = poly_model.predict(features)

            evaluate_model(dataset_dir, y, predictions, model_type, feature_name)

    feature_selection = "PCA Combined"

    for model_type in models:
        model_name_full = model_name(feature_selection, model_type)
        model_dir = os.path.join(dataset_dir, "models")
        poly_model = load_sklearn_model(model_dir, model_name_full)

        print(f"Evaluating {model_type} with {feature_selection}...")

        pca_amplitude_extractor = load_sklearn_model(model_dir, feature_extraction_name("PCA Amplitude"))
        pca_phase_extractor = load_sklearn_model(model_dir, feature_extraction_name("PCA Phase"))

        features_amplitude = pca_amplitude_extractor.dimensionality_reduction()
        features_phase = pca_phase_extractor.dimensionality_reduction()

        num_of_features = int(N_COMPONENTS / 2)
        features_combined = np.concatenate([features_phase[:, :num_of_features], features_amplitude[:, :num_of_features]], axis=1)

        predictions = poly_model.predict(features_combined)

        evaluate_model(dataset_dir, y, predictions, model_type, feature_selection)

    results = pd.read_csv(os.path.join(dataset_dir, "results_learned_farm.csv"))
    plot_accuracy_mae(results)