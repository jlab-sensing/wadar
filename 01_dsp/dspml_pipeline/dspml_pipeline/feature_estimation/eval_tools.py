from dspml_pipeline.feature_estimation.ridge_regression import RidgeRegression
from dspml_pipeline.feature_estimation.random_forest import RandomForest
from dspml_pipeline.feature_estimation.xgboost_tree import XGBoostTree
from dspml_pipeline.feature_estimation.svr import SVRRegression
from dspml_pipeline.results import update_results
from dspml_pipeline.feature_estimation.mlp import MLPRegression
from dspml_pipeline.parameters import num2label

from sklearn.metrics import mean_absolute_error, mean_squared_error, accuracy_score
import numpy as np
import matplotlib.pyplot as plt
from dspml_pipeline.results import load_results, display_feature_results

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
    
    # # ---- Plot: Labels vs Predictions ----
    # plt.figure(figsize=(6, 6))
    # plt.scatter(validation_labels, val_predictions, alpha=0.6, edgecolors='k')
    # plt.plot(
    #     [min(validation_labels), max(validation_labels)],
    #     [min(validation_labels), max(validation_labels)],
    #     'r--', lw=2
    # )
    # plt.xlabel("True Labels")
    # plt.ylabel("Predictions")
    # plt.title(f"{feature_name}: True vs Predicted")
    # plt.grid(True, linestyle="--", alpha=0.5)
    # plt.show()

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

def validate_classical_models(validation_dir, validation_features, validation_labels, feature_name, models):
    for model_name, model in models.items():
        if model is not None:
            val_predictions = model.estimate(validation_features)
            metrics = compute_metrics(validation_dir, validation_labels, feature_name, val_predictions)
            update_results(validation_dir, feature_name, model_name, metrics)

def classical_models_full_monty(training_dir:str, training_labels, validation_dir, validation_labels, 
                                tune_model_params:bool, training_features, validation_features, feature_name):
    ridgeRegressor, randomForest, gbTree, svr = evaluate_classic_models(training_dir, training_features, training_labels, tune_model_params, feature_name)
    results_df_amp = load_results(training_dir)
    display_feature_results(feature_name, results_df_amp)

    # Predict on the validation dataset using all trained models for {}
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
    results_df_amp = load_results(validation_dir)
    display_feature_results(feature_name, results_df_amp)

def end_to_end_model_validation(training_dir:np.ndarray,
                                validation_features:np.ndarray, 
                                validation_labels:np.ndarray, validation_directory:str, 
                                model_class:type, model_name:str):
    _, metrics = model_class.full_monty()
    update_results(training_dir, "End-to-end", model_name, metrics)
    y_pred = model_class.estimate(X_complex=validation_features)
    metrics = compute_metrics(validation_target_dir=validation_directory,
                                  validation_labels=validation_labels,
                                  feature_name="End-to-end",
                                  val_predictions=y_pred)
    update_results(validation_directory, "End-to-end", model_name, metrics)

def show_results_summary(feature_type, training_dir, validation_dir):
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