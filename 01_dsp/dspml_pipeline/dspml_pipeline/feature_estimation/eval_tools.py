from dspml_pipeline.feature_estimation.ridge_regression import RidgeRegression
from dspml_pipeline.feature_estimation.random_forest import RandomForest
from dspml_pipeline.feature_estimation.xgboost_tree import XGBoostTree
from dspml_pipeline.feature_estimation.svr import SVRRegression
from dspml_pipeline.results import update_results
from dspml_pipeline.feature_estimation.mlp import MLPRegression
from dspml_pipeline.end_to_end_estimation.cnn import CNNEstimator
from dspml_pipeline.end_to_end_estimation.lstm import LSTMEstimator
from dspml_pipeline.end_to_end_estimation.transformer import TransformerEstimator
from dspml_pipeline.parameters import num2label

from sklearn.metrics import mean_absolute_error, mean_squared_error, accuracy_score
import numpy as np
import matplotlib.pyplot as plt
from dspml_pipeline.results import load_results, display_feature_results

def compute_metrics(validation_labels:np.ndarray, val_predictions:np.ndarray):
    """
    Calculate evaluation metrics for validation estimation.

    Args:
        validation_target_dir (str):    Directory for validation results
        validation_labels (np.ndarray): True labels for validation data
        feature_name (str):             Name of feature for documentation
        val_predictions (np.ndarray):   Predicted values from the model

    Returns:
        metrics (dict): Dictionary containing MAE, RMSE, accuracy, inference time, and training time
    """

    # Compute metrics
    mae = mean_absolute_error(validation_labels, val_predictions)
    rmse = np.sqrt(mean_squared_error(validation_labels, val_predictions))
    val_true_labels = [num2label(label) for label in validation_labels]
    val_pred_labels = [num2label(pred) for pred in val_predictions]
    accuracy = np.mean([pred == true for pred, true in zip(val_pred_labels, val_true_labels)])

    # Inference and training time not evaluated for validation.
    inference_time = np.nan     
    training_time = np.nan

    metrics = {
        "mae": mae,
        "rmse": rmse,
        "accuracy": accuracy,
        "inference_time": inference_time,
        "training_time": training_time
    }

    return metrics

def evaluate_classic_models(training_dir:str, training_features:np.ndarray, training_labels:np.ndarray, tune_model_params:bool, feature_name:str):
    """
    Train and evaluate all classical models using training data.
    
    Args:
        training_dir (str):                 Directory of training data
        training_features (np.ndarray):     Handcrafted or learned features
        training_labels (np.ndarray):       Labels of training data
        tune_model_params (bool):           Set to false to disable model tuning
        feature_name (str):                 Name of feature for documentation

    Returns:
        ridgeRegressor (RidgeRegression)    Trained RidgeRegression class
        randomForest (RandomForest)         Trained RandomForest class
        gbTree (XGBoostTree)                Trained XGBoost class
        svr (SVRRegression)                 Trained SVRRegression class
    """
    
    ridgeRegressor = RidgeRegression()
    ridgeModel, metrics = ridgeRegressor.full_monty(training_features, training_labels)
    update_results(training_dir, feature_name, f"Ridge Regression Degree", metrics)

    # ==

    randomForest = RandomForest(tune_model_params=tune_model_params)
    model, metrics = randomForest.full_monty(training_features, training_labels)
    update_results(training_dir, feature_name, "Random Forest", metrics)

    # ==

    gbTree = XGBoostTree(tune_model_params)
    model, metrics = gbTree.full_monty(training_features, training_labels)
    update_results(training_dir, feature_name, "Gradient Boosted Tree", metrics)

    # ==

    svr = SVRRegression(tune_model_params)
    model, metrics = svr.full_monty(training_features, training_labels)
    update_results(training_dir, feature_name, "SVR", metrics)

    return ridgeRegressor, randomForest, gbTree, svr


def validate_classical_models(validation_dir: str, validation_features: np.ndarray, validation_labels: np.ndarray, feature_name: str, models: dict):
    """
    Run validation for all classical models and update results.

    Args:
        validation_dir (str):             Directory for validation results
        validation_features (np.ndarray): Features for validation data
        validation_labels (np.ndarray):   True labels for validation data
        feature_name (str):               Name of feature for documentation
        models (dict):                    Dictionary of model name to trained model instance
    """

    # Iterate through models dictionary and call their respective estimation functions
    for model_name, model in models.items():
        if model is not None:
            val_predictions = model.estimate(validation_features)
            metrics = compute_metrics(validation_labels, val_predictions)
            update_results(validation_dir, feature_name, model_name, metrics)

def classical_models_full_monty(training_dir: str, training_labels: np.ndarray, validation_dir: str, validation_labels: np.ndarray,
                                tune_model_params: bool, training_features: np.ndarray, validation_features: np.ndarray, feature_name: str):
    """
    Train, evaluate, and validate all classical models, and display results.

    Args:
        training_dir (str):                 Directory of training data
        training_labels (np.ndarray):       Labels of training data
        validation_dir (str):               Directory for validation results
        validation_labels (np.ndarray):     True labels for validation data
        tune_model_params (bool):           Set to false to disable model tuning
        training_features (np.ndarray):     Features for training data
        validation_features (np.ndarray):   Features for validation data
        feature_name (str):                 Name of feature for documentation
    """

    # Train all the classical models
    ridgeRegressor, randomForest, gbTree, svr = evaluate_classic_models(
        training_dir, training_features, training_labels, tune_model_params, feature_name
    )

    # Predict on the validation dataset using all trained models
    models_amp = {
        "Ridge Regression": ridgeRegressor,
        "Random Forest": randomForest,
        "Gradient Boosting": gbTree,
        "SVR": svr
    }
    validate_classical_models(
        validation_dir=validation_dir,
        validation_features=validation_features,
        validation_labels=validation_labels,
        feature_name=feature_name,
        models=models_amp
    )

def evaluate_deep_models(training_dir: str, training_features: np.ndarray, training_labels: np.ndarray, feature_name: str):
    """
    Train and evaluate all deep learning models using training data.
    
    Args:
        training_dir (str):                 Directory of training data
        training_features (np.ndarray):     Complex radar data for end-to-end models
        training_labels (np.ndarray):       Labels of training data
        feature_name (str):                 Name of feature for documentation

    Returns:
        mlp (MLPRegression)                 Trained MLP class
    """
    
    # MLP on features
    mlp = MLPRegression()
    _, metrics = mlp.full_monty(training_features, training_labels)
    update_results(training_dir, feature_name, "Multi Layer Perceptron", metrics)

    return mlp


def validate_deep_models(validation_dir: str, validation_features: np.ndarray, validation_labels: np.ndarray, 
                        feature_name: str, models: dict):
    """
    Run validation for all deep learning models and update results.

    Args:
        validation_dir (str):             Directory for validation results
        validation_features (np.ndarray): Features for validation data (for MLP)
        validation_labels (np.ndarray):   True labels for validation data
        feature_name (str):               Name of feature for documentation
        models (dict):                    Dictionary of model name to trained model instance
    """

    # Validate MLP on features
    if "Multi Layer Perceptron" in models and models["Multi Layer Perceptron"] is not None:
        val_predictions = models["Multi Layer Perceptron"].estimate(validation_features)
        metrics = compute_metrics(validation_labels, val_predictions)
        update_results(validation_dir, feature_name, "Multi Layer Perceptron", metrics)


def deep_full_monty(training_dir: str, training_labels: np.ndarray, validation_dir: str, validation_labels: np.ndarray,
                   training_features: np.ndarray, validation_features: np.ndarray, feature_name: str):
    """
    Train, evaluate, and validate all deep learning models, and display results.

    Args:
        training_dir (str):                 Directory of training data
        training_labels (np.ndarray):       Labels of training data
        validation_dir (str):               Directory for validation results
        validation_labels (np.ndarray):     True labels for validation data
        training_features (np.ndarray):     Features for training data (for MLP)
        validation_features (np.ndarray):   Features for validation data (for MLP)
        feature_name (str):                 Name of feature for documentation
    """

    # Train all the deep learning models
    mlp = evaluate_deep_models(
        training_dir, training_features, training_labels, feature_name
    )

    # Predict on the validation dataset using all trained models
    models_deep = {
        "Multi Layer Perceptron": mlp,
    }
    validate_deep_models(
        validation_dir=validation_dir,
        validation_features=validation_features,
        validation_labels=validation_labels,
        feature_name=feature_name,
        models=models_deep,
    )

def end_to_end_model_validation(training_dir:np.ndarray, validation_features:np.ndarray, validation_labels:np.ndarray, validation_directory:str, 
                                model_class:type, model_name:str):
    """
    Perform end-to-end validation for a given model class.

    Args:
        training_dir (str):                 Directory for training results
        validation_features (np.ndarray):   Features for validation data
        validation_labels (np.ndarray):     True labels for validation data
        validation_directory (str):         Directory for validation results
        model_class (type):                 Model class with full_monty and estimate methods
        model_name (str):                   Name of the model for documentation
    """
    
    _, metrics = model_class.full_monty()
    update_results(training_dir, "End-to-end", model_name, metrics)
    y_pred = model_class.estimate(X_complex=validation_features)
    metrics = compute_metrics(
        validation_labels=validation_labels,
        val_predictions=y_pred
    )
    update_results(validation_directory, "End-to-end", model_name, metrics)

def show_results_summary(feature_type: str, training_dir: str, validation_dir: str):
    """
    Display a summary of training and validation results for a given feature type.

    Args:
        feature_type (str):   Name of the feature type for documentation
        training_dir (str):   Directory containing training results
        validation_dir (str): Directory containing validation results
    """

    print("="*40)
    print(f"Training Results for {feature_type}".center(40))
    print("="*40)
    results_df_amp = load_results(training_dir)
    display_feature_results(feature_type, results_df_amp)
    print("="*40)
    print(f"Validation Results for {feature_type}".center(40))
    print("="*40)
    results_df_amp = load_results(validation_dir)
    display_feature_results(feature_type, results_df_amp)