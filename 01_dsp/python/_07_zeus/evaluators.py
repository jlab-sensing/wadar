"""
Model Evaluation Utilities
Contains functions for evaluating different ML models and displaying results.
"""

import numpy as np
from sklearn.metrics import mean_absolute_error, r2_score, root_mean_squared_error
from _06_hermes.parameters import num2label
from _06_hermes.logger import update_results
from _04_athena.regression import polynomial_regression
from _04_athena.tree import train_gradient_boosted_tree, train_random_forest
import _04_athena.svr as svr
from _04_athena.multi_later_percepetron import MultiLayerPerceptron
from _04_athena.pretrained_cnn import PretrainedCNNRegressor
from _04_athena.cnn_models import CNN1D


def evaluate_model(results_file_name, dataset_dir, features_name, model_name, true_labels, model_predictions):
    """
    Evaluate model predictions and log results.
    
    Args:
        results_file_name: Name of CSV file to save results
        dataset_dir: Directory to save results
        features_name: Name of feature set used
        model_name: Name of the model
        true_labels: Ground truth labels
        model_predictions: Model predictions
        
    Returns:
        Dictionary of calculated metrics
    """
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
    """Display formatted metrics comparison between training and validation."""
    print("="*60)
    print(f"[METRICS] {model_name}")
    print(f"{'Metric':<12} {'[TRAINING]':<20} {'[VALIDATION]':<20}")
    print(f"{'MAE':<12} {training_metrics['mae']:<20.4f} {validation_metrics['mae']:<20.4f}")
    print(f"{'R2':<12} {training_metrics['r2']:<20.4f} {validation_metrics['r2']:<20.4f}")
    print(f"{'RMSE':<12} {training_metrics['rmse']:<20.4f} {validation_metrics['rmse']:<20.4f}")
    print(f"{'Accuracy':<12} {training_metrics['accuracy']:<20.4f} {validation_metrics['accuracy']:<20.4f}")
    print("="*60)


def evaluate_ridge_regression(training_dataset, validation_dataset, training_labels, validation_labels, 
                             chosen_training_feature_array, chosen_validation_feature_array, 
                             feature_type_name, zeus_params):
    """Evaluate Ridge regression models with different polynomial degrees."""
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


def evaluate_random_forest(training_dataset, validation_dataset, training_labels, validation_labels,
                          chosen_training_feature_array, chosen_validation_feature_array,
                          feature_type_name, zeus_params):
    """Evaluate Random Forest model."""
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


def evaluate_gradient_boosted_tree(training_dataset, validation_dataset, training_labels, validation_labels,
                                  chosen_training_feature_array, chosen_validation_feature_array,
                                  feature_type_name, zeus_params):
    """Evaluate Gradient Boosted Tree model."""
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


def evaluate_svr(training_dataset, validation_dataset, training_labels, validation_labels,
                chosen_training_feature_array, chosen_validation_feature_array,
                feature_type_name, zeus_params):
    """Evaluate Support Vector Regression model."""
    model_name = "Support Vector Regression"
    print(f"[INFO] Evaluating {model_name} model on {feature_type_name}...")

    best_params = svr.tune_svr(chosen_training_feature_array, training_labels)
    model, training_metrics = svr.svr_regression(
        feature_array=chosen_training_feature_array,
        labels=training_labels,
        C=best_params['C'],
        gamma=best_params['gamma'],
        epsilon=best_params['epsilon']
    )

    update_results(training_dataset, feature_type_name, model_name, training_metrics, "training_results.csv", verbose=False)
    svr_model_predictions = model.predict(chosen_validation_feature_array)

    validation_metrics = evaluate_model(
        results_file_name="validation_results.csv",
        dataset_dir=validation_dataset,
        features_name=feature_type_name,
        model_name=model_name,
        true_labels=validation_labels,
        model_predictions=svr_model_predictions
    )

    display_model_metrics(model_name="Support Vector Regression", 
                          training_metrics=training_metrics, 
                          validation_metrics=validation_metrics)


def evaluate_mlp(training_dataset, validation_dataset, training_labels, validation_labels,
                chosen_training_feature_array, chosen_validation_feature_array,
                feature_type_name, zeus_params):
    """Evaluate Multi-Layer Perceptron model."""
    model_name = "Multi-Layer Perceptron"
    print(f"[INFO] Evaluating {model_name} model on {feature_type_name}...")

    mlp = MultiLayerPerceptron(chosen_training_feature_array, training_labels)
    model, training_metrics = mlp.full_monty(epochs=zeus_params['models']['mlp']['epochs'])

    update_results(training_dataset, feature_type_name, model_name, training_metrics, "training_results.csv", verbose=False)
    mlp_model_predictions = mlp.predict(chosen_validation_feature_array)

    validation_metrics = evaluate_model(
        results_file_name="validation_results.csv",
        dataset_dir=validation_dataset,
        features_name=feature_type_name,
        model_name=model_name,
        true_labels=validation_labels,
        model_predictions=mlp_model_predictions
    )

    display_model_metrics(model_name="Multi-Layer Perceptron", 
                          training_metrics=training_metrics, 
                          validation_metrics=validation_metrics)


def evaluate_cnn_regressor(training_dataset, validation_dataset, X_train, y_train, X_val, y_val,
                          feature_type_name, zeus_params):
    """Evaluate end-to-end CNN regressor model."""
    print(f"\n[INFO] Evaluating CNN regressor on raw radar data...")

    cnn_params = zeus_params['models']['cnn_regressor']
    epochs = cnn_params.get('epochs', 30)
    img_size = tuple(cnn_params.get('img_size', [160, 160]))
    batch_size = cnn_params.get('batch_size', 32)
    output_dir = cnn_params.get('output_dir', './cnn_temp_images_regressor')
    verbose = zeus_params.get('advanced', {}).get('verbose', False)

    # Initialize CNN regressor with raw radar data
    cnn_regressor = PretrainedCNNRegressor(
        X=X_train, 
        y=y_train, 
        output_dir=output_dir,
        img_size=img_size,
        batch_size=batch_size,
        verbose=verbose
    )

    # Train the model
    print(f"[INFO] Training CNN regressor for {epochs} epochs...")
    model, training_metrics = cnn_regressor.full_monty(epochs=epochs)

    # Make predictions on validation set
    print(f"[INFO] Making CNN regressor predictions on validation set...")
    cnn_predictions = cnn_regressor.predict(X_val)

    # Evaluate validation performance
    validation_metrics = evaluate_model(
        results_file_name="validation_results.csv",
        dataset_dir=validation_dataset,
        features_name=feature_type_name,
        model_name="CNN Regressor",
        true_labels=y_val,
        model_predictions=cnn_predictions.flatten() if len(cnn_predictions.shape) > 1 else cnn_predictions
    )

    # Log training metrics
    update_results(training_dataset, feature_type_name, "CNN Regressor", training_metrics, "training_results.csv", verbose=False)

    display_model_metrics(model_name=feature_type_name, 
                          training_metrics=training_metrics, 
                          validation_metrics=validation_metrics)

    # Save the trained model if requested
    if zeus_params['models']['cnn_regressor'].get('save_model', False):
        save_dir = zeus_params['models']['cnn_regressor'].get('save_directory', './models')
        cnn_regressor.save_model(save_dir, "cnn_regressor_model.keras")
        print(f"[INFO] CNN regressor model saved to {save_dir}")

    
    return cnn_regressor, training_metrics, validation_metrics


def evaluate_cnn1d(training_dataset, validation_dataset, X_train, y_train, X_val, y_val, 
                   feature_type_name, zeus_params):
    """
    Evaluate 1D CNN model on raw radar data.
    
    Args:
        training_dataset: Training dataset directory
        validation_dataset: Validation dataset directory  
        X_train: Training radar data (complex)
        y_train: Training labels
        X_val: Validation radar data (complex)
        y_val: Validation labels
        feature_type_name: Name of feature type for logging
        zeus_params: Zeus configuration parameters
    
    Returns:
        Trained CNN1D model and metrics
    """
    print(f"\n[INFO] Evaluating 1D CNN on raw radar data...")
    
    # Get CNN1D parameters
    cnn1d_params = zeus_params['models'].get('cnn1d', {})
    epochs = cnn1d_params.get('epochs', 30)
    batch_size = cnn1d_params.get('batch_size', 16)
    save_model = cnn1d_params.get('save_model', False)
    save_directory = cnn1d_params.get('save_directory', './models')
    
    # Initialize and train 1D CNN
    cnn1d = CNN1D(X_train, y_train)
    
    # Train model and get cross-validation metrics
    model, training_metrics = cnn1d.full_monty(epochs=epochs, batch_size=batch_size)
    
    # Make predictions on validation set
    print(f"[INFO] Making 1D CNN predictions on validation set...")
    y_pred_val = cnn1d.predict(X_val)
    
    # Evaluate validation performance
    validation_metrics = evaluate_model(
        results_file_name="validation_results.csv",
        dataset_dir=validation_dataset,
        features_name=feature_type_name,
        model_name="1D CNN",
        true_labels=y_val,
        model_predictions=y_pred_val
    )
    
    # Log training metrics
    update_results(
        training_dataset, 
        feature_type_name, 
        "1D CNN", 
        training_metrics, 
        "training_results.csv", 
        verbose=False
    )
    
    # Display results
    display_model_metrics(
        model_name=feature_type_name, 
        training_metrics=training_metrics, 
        validation_metrics=validation_metrics
    )
    
    # Save model if requested
    if save_model:
        try:
            cnn1d.save_model(save_directory, "cnn1d_regressor.keras")
            print(f"[INFO] 1D CNN model saved to {save_directory}")
        except Exception as e:
            print(f"[WARNING] Failed to save 1D CNN model: {e}")
    
    return cnn1d, training_metrics, validation_metrics

def evaluate_all_models(zeus_params, training_dataset, validation_dataset, training_labels, validation_labels, 
                       training_features, validation_features, feature_type_name, 
                       X_train=None, y_train=None, X_val=None, y_val=None):
    """Evaluate all enabled models on given feature set (excluding end-to-end CNN models)."""
    model_configs = zeus_params['models']
    
    if model_configs['ridge_regression']['enabled']:
        evaluate_ridge_regression(training_dataset, validation_dataset, training_labels, validation_labels,
                                 training_features, validation_features, feature_type_name, zeus_params)

    if model_configs['random_forest']['enabled']:
        evaluate_random_forest(training_dataset, validation_dataset, training_labels, validation_labels,
                              training_features, validation_features, feature_type_name, zeus_params)

    if model_configs['gradient_boosted_tree']['enabled']:
        evaluate_gradient_boosted_tree(training_dataset, validation_dataset, training_labels, validation_labels,
                                      training_features, validation_features, feature_type_name, zeus_params)

    if model_configs['svr']['enabled']:
        evaluate_svr(training_dataset, validation_dataset, training_labels, validation_labels,
                    training_features, validation_features, feature_type_name, zeus_params)
        
    if model_configs['mlp']['enabled']:
        evaluate_mlp(training_dataset, validation_dataset, training_labels, validation_labels,
                    training_features, validation_features, feature_type_name, zeus_params)
