from dspml_pipeline.feature_estimation.ridge_regression import RidgeRegression
from dspml_pipeline.feature_estimation.random_forest import RandomForest
from dspml_pipeline.feature_estimation.xgboost_tree import XGBoostTree
from dspml_pipeline.feature_estimation.svr import SVRRegression
from dspml_pipeline.results import update_results
from dspml_pipeline.feature_estimation.mlp import MLPRegression

from sklearn.metrics import mean_absolute_error, mean_squared_error, accuracy_score
from dspml_pipeline.parameters import num2label
import numpy as np

def evaluate_classic_models(target_dir, feature_array, labels, tune_model_params, feature_name):
    
    ridgeRegressor = RidgeRegression()
    ridgeModel, metrics = ridgeRegressor.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, f"Ridge Regression Degree", metrics)

    # ==

    randomForest = RandomForest(tune_model_params=tune_model_params)
    model, metrics = randomForest.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, "Random Forest", metrics)

    # ==

    gbTree = XGBoostTree(tune_model_params)
    model, metrics = gbTree.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, "Gradient Boosted Tree", metrics)

    # ==

    svr = SVRRegression(tune_model_params)
    model, metrics = svr.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, "SVR", metrics)

    return ridgeRegressor, randomForest, gbTree, svr

def evaluate_deep_models(target_dir, feature_array, labels, feature_name):
    mlp = MLPRegression()
    _, metrics = mlp.full_monty(feature_array, labels)
    update_results(target_dir, feature_name, "Multi Layer Pecepetron", metrics)

def compute_metrics(validation_target_dir, validation_labels, feature_name, val_predictions):
    # Calculate evaluation metrics for validation estimation
    mae = mean_absolute_error(validation_labels, val_predictions)
    rmse = np.sqrt(mean_squared_error(validation_labels, val_predictions))

    val_true_labels = [num2label(label) for label in validation_labels]
    val_pred_labels = [num2label(pred) for pred in val_predictions]
    accuracy = np.mean([pred == true for pred, true in zip(val_pred_labels, val_true_labels)])

    inference_time = np.nan     # Inference and training time not evaluated for validation.
    training_time = np.nan

    metrics = {
        "mae": mae,
        "rmse": rmse,
        "accuracy": accuracy,
        "inference_time": inference_time,
        "training_time": training_time
    }

    return metrics

def validate_classical_models(validation_target_dir, validation_feature_array, validation_labels, feature_name, models):
    for model_name, model in models.items():
        val_predictions = model.estimate(validation_feature_array)
        metrics = compute_metrics(validation_target_dir, validation_labels, feature_name, val_predictions)
        update_results(validation_target_dir, feature_name, model_name, metrics)