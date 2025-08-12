"""
Tools to consolidate results.
"""

import logging
logger = logging.getLogger(__name__)

import os
from datetime import datetime
import numpy as np
import pandas as pd
import sys
from pathlib import Path

from .plotting_tools.data_plotting import plot_accuracy_mae


def update_results(target_dir: str, feature_name: str, model_name: str, 
    metrics: dict, results_file_name: str = "results.csv"
):
    """
    Update the results CSV file with the latest model performance metrics.
    Used to maintain a log of model performance across different feature sets 
    and model types.

    Parameters:
        target_dir: Directory where the results CSV file is stored
        feature_name: Name/description of the feature set used
        model_name: Name of the model (e.g., 'RandomForest', 'SVM', etc.)
        metrics: Dictionary containing performance metrics with keys:
                - 'mae': Mean Absolute Error
                - 'rmse': Root Mean Square Error  
                - 'accuracy': Model accuracy
                - 'inference_time': Time taken for inference (seconds)
                - 'training_time': Time taken for training (seconds)
        results_file_name: Name of the results CSV file
    """

    if not target_dir or not os.path.exists(target_dir):
        logger.error(f"Dataset directory does not exist: {target_dir}")
        sys.exit(1)
    
    required_metrics = ['mae', 'rmse', 'accuracy', 'inference_time', 'training_time']
    missing_metrics = [metric for metric in required_metrics if metric not in metrics]
    if missing_metrics:
        logger.error(f"Missing required metrics: {missing_metrics}")
        sys.exit(1)

    # Extract metrics 
    mae = float(metrics["mae"])
    rmse = float(metrics["rmse"])
    accuracy = float(metrics["accuracy"])
    inference_time = float(metrics["inference_time"])
    training_time = float(metrics["training_time"])


    results_file_path = Path(target_dir) / results_file_name
    
    if results_file_path.exists():
        results_df = pd.read_csv(results_file_path)
        logger.info(f"Updating results file with {len(results_df)} entries")
    else:
        results_df = _create_empty_results_df()
        logger.info("Creating new results file")

    # Check if entry already exists
    existing_mask = (results_df["Model"] == model_name) & (results_df["Feature"] == feature_name)
    existing_idx = results_df[existing_mask].index

    # Create new row with all metrics
    new_row = {
        "Feature": feature_name,
        "Model": model_name,
        "Accuracy": accuracy,
        "MAE": mae,
        "RMSE": rmse,
        "Training Time": training_time,
        "Inference Time": inference_time,
        "Last Updated": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }

    if len(existing_idx) > 0:
        # Update existing entry
        results_df.loc[existing_idx[0]] = new_row
    else:
        # Add new entry
        results_df = pd.concat([results_df, pd.DataFrame([new_row])], ignore_index=True)

    # Save results
    results_df.to_csv(results_file_path, index=False)


def _create_empty_results_df():
    """
    Create an empty DataFrame with the standard results format.
    """
    return pd.DataFrame(columns=[
        "Feature", "Model", "Accuracy", "MAE", "RMSE", "Training Time", "Inference Time", "Last Updated"
    ])


def load_results(dataset_dir: str, results_file_name: str = "results.csv"):
    """
    Load model results from CSV file.
    
    Components:
        dataset_dir: Directory containing the results file
        results_file_name: Name of the results CSV file
        
    Returns:
        DataFrame containing results
    """

    file_path = os.path.join(dataset_dir, results_file_name)
    
    if not Path(file_path).exists():
        logger.error(f"Results file not found: {file_path}")
        sys.exit(1)
        
    results_df = pd.read_csv(file_path)
    return results_df

def display_feature_results(feature_name, results_df, show_plot=False):
    handcrafted_results = results_df[results_df['Feature'] == feature_name]

    print(handcrafted_results)

    plot_accuracy_mae(handcrafted_results, figure_title=feature_name, show_plot=show_plot)