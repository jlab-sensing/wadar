# Zeus: End-to-End Evaluation Script

import os
import sys
import numpy as np
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _01_gaia.dataset import Dataset, combine_datasets
from _03_hephaestus.feature_tools import FeatureTools
from _03_hephaestus import feature_tools
from _04_athena.regression import polynomial_regression
from _06_hermes.logger import update_results
from sklearn.metrics import mean_absolute_error, r2_score, root_mean_squared_error
from _06_hermes.parameters import num2label
import pandas as pd
import yaml
from _04_athena.tree import train_gradient_boosted_tree, train_random_forest
from _03_hephaestus.pca_tools import PCAProcessor

def evaluate_model(results_file_name, dataset_dir, features_name, model_name, true_labels, model_predictions):
    mae = mean_absolute_error(true_labels, model_predictions)
    r2 = r2_score(true_labels, model_predictions)
    rmse = root_mean_squared_error(true_labels, model_predictions)

    y_labels = [num2label(label) for label in true_labels]
    predictions_labels = [num2label(pred) for pred in model_predictions]
    accuracy = np.mean(np.array(y_labels) == np.array(predictions_labels))

    metrics = {
        "mae": mae,
        "r2": r2,
        "rmse": rmse,
        "accuracy": accuracy,
        "training_time": 0,
        "inference_time": 0,
    }

    update_results(dataset_dir, features_name, model_name, metrics, results_file_name, verbose=False)

    return metrics

def display_model_metrics(model_name, training_metrics, validation_metrics):
    print("="*60)
    print(f"[METRICS] {model_name}")
    print(f"{'Metric':<12} {'[TRAINING]':<20} {'[VALIDATION]':<20}")
    print(f"{'MAE':<12} {training_metrics['mae']:<20.4f} {validation_metrics['mae']:<20.4f}")
    print(f"{'R2':<12} {training_metrics['r2']:<20.4f} {validation_metrics['r2']:<20.4f}")
    print(f"{'RMSE':<12} {training_metrics['rmse']:<20.4f} {validation_metrics['rmse']:<20.4f}")
    print(f"{'Accuracy':<12} {training_metrics['accuracy']:<20.4f} {validation_metrics['accuracy']:<20.4f}")
    print("="*60)

def evaluate_ridge_regression(training_dataset, validation_dataset, training_labels, validation_labels, chosen_training_feature_array, chosen_validation_feature_array, feature_type_name, zeus_params):

    ridge_params = zeus_params['models']['ridge_regression']
    degrees = ridge_params.get('degrees', [1, 2, 3])
    alpha = ridge_params.get('alpha', 1.0)

    for degree in degrees:

        model_name = f"Ridge Regression Degree {degree}"
        print(f"[INFO] Evaluating {model_name} model on {feature_type_name}...")

        poly_model, poly_metrics = polynomial_regression(
            feature_array=chosen_training_feature_array,
            labels=training_labels,
            degree=degree,
            alpha=alpha
        )

        update_results(training_dataset, feature_type_name, model_name, poly_metrics, "training_results.csv", verbose=False)

        poly_model_predictions = poly_model.predict(chosen_validation_feature_array)

        metrics = evaluate_model(
            results_file_name="validation_results.csv",
            dataset_dir=validation_dataset,
            features_name=feature_type_name,
            model_name=model_name,
            true_labels=validation_labels,
            model_predictions=poly_model_predictions
        )
        
        display_model_metrics(model_name, poly_metrics, metrics)

def evaluate_random_forest(training_dataset, validation_dataset, training_labels, validation_labels, chosen_training_feature_array, chosen_validation_feature_array, feature_type_name, zeus_params):

    model_name = "Random Forest"
    print(f"[INFO] Evaluating {model_name} model on {feature_type_name}...")

    rf_params = zeus_params['models']['random_forest']
    n_estimators = rf_params.get('n_estimators', 100)

    model, training_metrics = train_random_forest(
        feature_array=chosen_training_feature_array,
        labels=training_labels,
        n_estimators=n_estimators
    )

    update_results(training_dataset, feature_type_name, model_name, training_metrics, "training_results.csv", verbose=False)

    rf_model_predictions = model.predict(chosen_validation_feature_array)

    validation_metrics = evaluate_model(
        results_file_name="validation_results.csv",
        dataset_dir=validation_dataset,
        features_name=feature_type_name,
        model_name=model_name,
        true_labels=validation_labels,
        model_predictions=rf_model_predictions
    )

    display_model_metrics(model_name="Random Forest", 
                          training_metrics=training_metrics, 
                          validation_metrics=validation_metrics)
    
def evaluate_gradient_boosted_tree(training_dataset, validation_dataset, training_labels, validation_labels, chosen_training_feature_array, chosen_validation_feature_array, feature_type_name, zeus_params):
    model_name = "Gradient Boosted Tree"
    print(f"[INFO] Evaluating {model_name} model on {feature_type_name}...")

    gbt_params = zeus_params['models']['gradient_boosted_tree']
    n_estimators = gbt_params.get('n_estimators', 100)

    model, training_metrics = train_gradient_boosted_tree(
        feature_array=chosen_training_feature_array,
        labels=training_labels,
        n_estimators=n_estimators
    )

    update_results(training_dataset, feature_type_name, model_name, training_metrics, "training_results.csv", verbose=False)

    gbt_model_predictions = model.predict(chosen_validation_feature_array)

    validation_metrics = evaluate_model(
        results_file_name="validation_results.csv",
        dataset_dir=validation_dataset,
        features_name=feature_type_name,
        model_name=model_name,
        true_labels=validation_labels,
        model_predictions=gbt_model_predictions
    )

    display_model_metrics(model_name="Gradient Boosted Tree", 
                          training_metrics=training_metrics, 
                          validation_metrics=validation_metrics)

def evaluate_all_models(zeus_params,
                        training_dataset, 
                        validation_dataset, 
                        training_labels, 
                        validation_labels, 
                        training_features, 
                        validation_features,
                        feature_type_name):
    
    RIDGE_REGRESSION_ENABLED = zeus_params['models']['ridge_regression']['enabled']
    RANDOM_FOREST_ENABLED = zeus_params['models']['random_forest']['enabled']
    GRADIENT_BOOSTED_TREE_ENABLED = zeus_params['models']['gradient_boosted_tree']['enabled']
        
    if RIDGE_REGRESSION_ENABLED:
        evaluate_ridge_regression(training_dataset=training_dataset,
                                      validation_dataset=validation_dataset,
                                      training_labels=training_labels,
                                      validation_labels=validation_labels,
                                      chosen_training_feature_array=training_features,
                                      chosen_validation_feature_array=validation_features,
                                      feature_type_name=feature_type_name,
                                      zeus_params=zeus_params)

    if RANDOM_FOREST_ENABLED:
        evaluate_random_forest(training_dataset=training_dataset,
                                    validation_dataset=validation_dataset,
                                    training_labels=training_labels,
                                    validation_labels=validation_labels,
                                    chosen_training_feature_array=training_features,
                                    chosen_validation_feature_array=validation_features,
                                    feature_type_name=feature_type_name,
                                    zeus_params=zeus_params)

    if GRADIENT_BOOSTED_TREE_ENABLED:
        evaluate_gradient_boosted_tree(training_dataset=training_dataset,
                                           validation_dataset=validation_dataset,
                                           training_labels=training_labels,
                                           validation_labels=validation_labels,
                                           chosen_training_feature_array=training_features,
                                           chosen_validation_feature_array=validation_features,
                                           feature_type_name=feature_type_name,
                                           zeus_params=zeus_params)

if __name__ == "__main__":

    with open("zeus_params.yaml", "r") as f:
        zeus_params = yaml.safe_load(f)

    raw_training_datasets = zeus_params['data']['training']['raw_datasets']
    target_training_datasets = zeus_params['data']['training']['target_dataset']
    raw_validation_datasets = zeus_params['data']['validation']['raw_datasets']
    target_validation_dataset = zeus_params['data']['validation']['target_dataset']

    new_dataset = zeus_params['data']['new_dataset']

    # ====================================================

    # Prepare the training and validation datasets
    training_dataset = target_training_datasets
    validation_dataset = target_validation_dataset

    # Combine datasets
    if new_dataset:
        combine_datasets(raw_training_datasets, target_training_datasets, verbose=True)
        combine_datasets(raw_validation_datasets, target_validation_dataset, verbose=True)
    
    # Load the training and validation datasets
    training_frame_loader = FrameLoader(training_dataset, new_dataset=new_dataset, ddc_flag=True, verbose=new_dataset)
    X_train, y_train = training_frame_loader.X, training_frame_loader.y
    validation_frame_loader = FrameLoader(validation_dataset, new_dataset=new_dataset, ddc_flag=True, verbose=new_dataset)
    X_val, y_val = validation_frame_loader.X, validation_frame_loader.y

    # ====================================================

    # Assembling handcrafted features

    TEST_HANDCRAFTED_FEATURES = zeus_params['features']['handcrafted']['enabled']
    N_FEATURES = zeus_params['features']['handcrafted']['n_features']

    if TEST_HANDCRAFTED_FEATURES:

        print("[INFO] Assembling handcrafted features...", zeus_params)

        # Load or create features
        if new_dataset:
            handcrafted_feature_engineering = FeatureTools(X_train)
            training_feature_table = handcrafted_feature_engineering.feature_full_monty(y_train, training_dataset)
            handcrafted_feature_engineering = FeatureTools(X_val)
            validation_feature_table = handcrafted_feature_engineering.feature_full_monty(y_val, validation_dataset)
        training_feature_table, training_feature_array, training_feature_names, training_labels = feature_tools.load_feature_table(training_dataset)
        validation_feature_table, validation_feature_array, validation_feature_names, validation_labels = feature_tools.load_feature_table(validation_dataset)

        # Best features based on Mutual Information, for all models

        mi_training_feature_table, _ = feature_tools.mutual_info_minimize_features(training_feature_table, top_n=N_FEATURES)
        feature_tools.save_feature_table(mi_training_feature_table, training_dataset, "mi_features.csv")
        _, mi_feature_array, mi_feature_names, mi_labels = feature_tools.load_feature_table(training_dataset, "mi_features.csv")
        mi_validation_feature_table = pd.concat(
            [validation_feature_table[mi_feature_names], validation_feature_table[['Label']]], axis=1
        )
        feature_tools.save_feature_table(mi_validation_feature_table, validation_dataset, "mi_features.csv")
        _, mi_validation_feature_array, mi_validation_feature_names, mi_validation_labels = feature_tools.load_feature_table(validation_dataset, "mi_features.csv")

        # Best features based on Correlation, for linear models

        corr_training_feature_table, _ = feature_tools.correlation_minimize_features(training_feature_table, top_n=N_FEATURES)
        feature_tools.save_feature_table(corr_training_feature_table, training_dataset, "corr_features.csv")
        _, corr_feature_array, corr_feature_names, corr_labels = feature_tools.load_feature_table(training_dataset, "corr_features.csv")
        corr_validation_feature_table = pd.concat(
            [validation_feature_table[corr_feature_names], validation_feature_table[['Label']]], axis=1
        )
        feature_tools.save_feature_table(corr_validation_feature_table, validation_dataset, "corr_features.csv")
        _, corr_validation_feature_array, corr_validation_feature_names, corr_validation_labels = feature_tools.load_feature_table(validation_dataset, "corr_features.csv")

        if zeus_params['features']['handcrafted']['chosen_method'] == 'mutual_info':
            chosen_training_feature_array = mi_feature_array
            chosen_validation_feature_array = mi_validation_feature_array 
            chosen_training_feature_names = mi_feature_names
            chosen_validation_feature_names = mi_validation_feature_names
            training_labels = mi_labels
            validation_labels = mi_validation_labels
        elif zeus_params['features']['handcrafted']['chosen_method'] == 'correlation':
            chosen_training_feature_array = corr_feature_array
            chosen_validation_feature_array = corr_validation_feature_array 
            chosen_training_feature_names = corr_feature_names
            chosen_validation_feature_names = corr_validation_feature_names
            training_labels = corr_labels
            validation_labels = corr_validation_labels
        else:
            print("[WARNING] No feature selection method chosen, using all features.")
            chosen_training_feature_array = training_feature_array
            chosen_validation_feature_array = validation_feature_array
            chosen_training_feature_names = training_feature_names
            chosen_validation_feature_names = validation_feature_names
            training_labels = training_labels
            validation_labels = validation_labels

        # Testing the feature table on enabled models

        evaluate_all_models(zeus_params=zeus_params,
                            training_dataset=training_dataset,
                            validation_dataset=validation_dataset,
                            training_labels=training_labels,
                            validation_labels=validation_labels,
                            training_features=chosen_training_feature_array,
                            validation_features=chosen_validation_feature_array,
                            feature_type_name="Handcrafted Features")
            
        print("[INFO] Handcrafted features evaluation completed.")

    # ====================================================

    # Assembling PCA-based features.

    if zeus_params['features']['pca']['enabled']:

        print("[INFO] Assembling PCA-based features...")

        X_train_amplitude = np.abs(X_train)
        X_val_amplitude = np.abs(X_val)
        X_train_phase = np.angle(X_train)
        X_val_phase = np.angle(X_val)

        pca_processor_amplitude = PCAProcessor(X_train_amplitude, n_components=zeus_params['features']['pca']['n_features'])
        pca_processor_phase = PCAProcessor(X_train_phase, n_components=zeus_params['features']['pca']['n_features'])

        features_amplitude_train = pca_processor_amplitude.dimensionality_reduction()
        features_amplitude_val = pca_processor_amplitude.transform(X_val_amplitude)

        features_phase_train = pca_processor_phase.dimensionality_reduction()
        features_phase_val = pca_processor_phase.transform(X_val_phase)

        features_combined_train = np.concatenate(
            [features_amplitude_train, features_phase_train], axis=1
        )
        features_combined_val = np.concatenate(
            [features_amplitude_val, features_phase_val], axis=1
        )

        print("[INFO] Evaluating PCA Amplitude features...")
        evaluate_all_models(zeus_params=zeus_params,
                            training_dataset=training_dataset,
                            validation_dataset=validation_dataset,
                            training_labels=y_train,
                            validation_labels=y_val,
                            training_features=features_amplitude_train,
                            validation_features=features_amplitude_val,
                            feature_type_name="PCA Amplitude")

        print("[INFO] Evaluating PCA Phase features...")
        evaluate_all_models(zeus_params=zeus_params,
                            training_dataset=training_dataset,
                            validation_dataset=validation_dataset,
                            training_labels=y_train,
                            validation_labels=y_val,
                            training_features=features_phase_train,
                            validation_features=features_phase_val,
                            feature_type_name="PCA Phase")
        
        print("[INFO] Evaluating PCA Combined features...")
        evaluate_all_models(zeus_params=zeus_params,
                            training_dataset=training_dataset,
                            validation_dataset=validation_dataset,
                            training_labels=y_train,
                            validation_labels=y_val,
                            training_features=features_combined_train,
                            validation_features=features_combined_val,
                            feature_type_name="PCA Combined")
        
        print("[INFO] PCA-based features evaluation completed.")

    # ====================================================