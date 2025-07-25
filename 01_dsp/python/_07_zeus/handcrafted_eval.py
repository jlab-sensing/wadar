# This script evaluates various regression machine learning models on a dataset with handcrafted features.

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
from pickle import dump, load
from sklearn.metrics import mean_absolute_error, r2_score
import numpy as np
from _06_hermes.parameters import num2label, RANDOM_SEED
import matplotlib.pyplot as plt
import time
from _05_apollo.viz_tools import plot_accuracy_mae

def load_model(models_dir, model_name):
    model_path = os.path.join(models_dir, model_name)
    with open(model_path, 'rb') as f:
        poly_model = load(f)
    return poly_model

def evaluate_sklearn_model(dataset_dir, model_name, feature_table, labels, models_dir, degree, model, trained_features_filename):
    """
    Evaluate a trained sklearn model on the provided feature array and labels.

    Args:
        dataset_dir (str):                  Directory where the dataset is stored.
        model_name (str):                   Name of the model.
        feature_array (np.ndarray):         Array of features of shape (samples, features).
        feature_names (list):               List of feature names.
        labels (np.ndarray):                Array of labels of shape (samples,).
        models_dir (str):                   Directory where the model is stored.
        degree (int):                       Degree of the polynomial features.
        model (sklearn model):              The trained sklearn model to evaluate.
        trained_features_filename (str):    Name of the file containing the features used for training.

    Returns:
        None:                       Updates the results CSV file with the model performance metrics.
    """

    model_trained_df = pd.read_csv(os.path.join(models_dir, trained_features_filename))
    trained_feature_names = [col for col in model_trained_df.columns if col != "Label"]
    feature_array_selected = feature_table[trained_feature_names].values

    predictions = model.predict(feature_array_selected)

    mae = mean_absolute_error(labels, predictions)
    rmse = np.sqrt(np.mean((labels - predictions) ** 2))
    r2 = r2_score(labels, predictions)

    y_labels = [num2label(label) for label in labels]
    predictions_labels = [num2label(pred) for pred in predictions]
    accuracy = np.mean(np.array(y_labels) == np.array(predictions_labels))

    update_results(
            "Handcrafted",
            model_name,
            accuracy,
            mae,
            rmse,
            r2,
            0,  # Training time not applicable here
            0,  # Inference time not applicable here
            dataset_dir,
            f"results_farm.csv"
        )

if __name__ == "__main__":

    # Validation dataset
    dataset_dir = "../data/training-dataset"
    new_dataset = False

    hydros = loader.FrameLoader(dataset_dir, new_dataset=new_dataset, ddc_flag=True)
    X, y = hydros.X, hydros.y

    if new_dataset:
        hephaestus_features = feature_tools.FeatureTools(X)
        feature_table = hephaestus_features.feature_full_monty(y, dataset_dir)

    feature_table, feature_array, feature_names, labels = feature_tools.load_feature_table(dataset_dir)

    models_dir = os.path.join(dataset_dir, "models")

    # ==============
    # Evaluating ridge regression models
    # ==============

    for degree in [1, 2, 3]:

        model_name = f"model_regression_degree_{degree}.pkl"

        poly_model = load_model(models_dir, model_name)

        print(f"Evaluating ridge regression model with degree {degree}...")

        feature_file_name = f"feature_linear_regression_{degree}_monte_carlo.csv"

        evaluate_sklearn_model(dataset_dir, f"Regression Degree {degree}", feature_table, labels, models_dir, degree, poly_model, feature_file_name)

    # ==============
    # Evaluating Random Forest models
    # ==============

    model_name = "model_random_forest.pkl"

    rf_model = load_model(models_dir, model_name)

    print(f"Evaluating random forest model...")

    feature_file_name = f"feature_random_forest_monte_carlo.csv"

    evaluate_sklearn_model(dataset_dir, "Random Forest", feature_table, labels, models_dir, None, rf_model, feature_file_name)

    # ==============
    # Evaluating Gradient Boosted Tree models
    # ==============

    model_name = "model_gradient_boosted_tree.pkl"

    gbt_model = load_model(models_dir, model_name)

    print(f"Evaluating gradient boosted tree model...")

    feature_file_name = f"feature_gradient_boosted_tree_monte_carlo.csv"

    evaluate_sklearn_model(dataset_dir, "Gradient Boosted Tree", feature_table, labels, models_dir, None, gbt_model, feature_file_name)

    # ==============
    # Evaluating Support Vector Regression models
    # ==============

    model_name = "model_svr.pkl"

    svr_model = load_model(models_dir, model_name)

    print(f"Evaluating support vector regression model...")

    feature_file_name = f"feature_svr_monte_carlo.csv"

    evaluate_sklearn_model(dataset_dir, "Support Vector Regression", feature_table, labels, models_dir, None, svr_model, feature_file_name)

    # ==============
    # Results
    # ==============

    results = pd.read_csv(os.path.join(dataset_dir, "results_farm.csv"))

    # Call the function
    plot_accuracy_mae(results)
