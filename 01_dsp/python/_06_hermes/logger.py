"""
Optimized logging utilities for model performance tracking and results management.

This module provides functionality to log, update, and manage model performance 
metrics in a structured CSV format for easy analysis and comparison.
"""

import os
import sys
from typing import Dict, Optional, Union, Any
from datetime import datetime
import numpy as np
import pandas as pd


def update_results(
    dataset_dir: str, 
    feature_name: str, 
    model_name: str, 
    metrics: Dict[str, Union[float, int]], 
    results_file_name: str = "model_results.csv",
    verbose: bool = True
) -> None:
    """
    Update the results CSV file with the latest model performance metrics.
    
    This function maintains a comprehensive log of model performance across
    different feature sets and model types, enabling easy comparison and tracking.

    Args:
        dataset_dir: Directory where the results CSV file is stored
        feature_name: Name/description of the feature set used
        model_name: Name of the model (e.g., 'RandomForest', 'SVM', etc.)
        metrics: Dictionary containing performance metrics with keys:
                - 'mae': Mean Absolute Error
                - 'rmse': Root Mean Square Error  
                - 'r2': R-squared score
                - 'accuracy': Model accuracy
                - 'inference_time': Time taken for inference (seconds)
                - 'training_time': Time taken for training (seconds)
        results_file_name: Name of the results CSV file
        verbose: Whether to print status messages

    Returns:
        None

    Raises:
        ValueError: If required metrics are missing
        OSError: If file operations fail
    """
    try:
        # Validate inputs
        if not dataset_dir or not os.path.exists(dataset_dir):
            raise ValueError(f"Dataset directory does not exist: {dataset_dir}")
        
        if not feature_name or not model_name:
            raise ValueError("Feature name and model name cannot be empty")
        
        required_metrics = ['mae', 'rmse', 'r2', 'accuracy', 'inference_time', 'training_time']
        missing_metrics = [metric for metric in required_metrics if metric not in metrics]
        if missing_metrics:
            raise ValueError(f"Missing required metrics: {missing_metrics}")

        # Extract metrics with type validation
        mae = float(metrics["mae"])
        rmse = float(metrics["rmse"])
        r2 = float(metrics["r2"])
        accuracy = float(metrics["accuracy"])
        inference_time = float(metrics["inference_time"])
        training_time = float(metrics["training_time"])

        if verbose:
            print(f"[INFO] Updating results for {model_name} with {feature_name} features")

        # Construct file path
        results_file_path = os.path.join(dataset_dir, results_file_name)
        
        # Load or create results DataFrame
        if os.path.exists(results_file_path):
            try:
                results_df = pd.read_csv(results_file_path)
                if verbose:
                    print(f"[INFO] Loaded existing results file with {len(results_df)} entries")
            except Exception as e:
                print(f"[WARNING] Failed to load existing results file: {e}")
                print("[INFO] Creating new results file")
                results_df = _create_empty_results_df()
        else:
            results_df = _create_empty_results_df()
            if verbose:
                print("[INFO] Created new results file")

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
            "R2": r2,
            "Training Time": training_time,
            "Inference Time": inference_time,
            "Last Updated": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }

        if len(existing_idx) > 0:
            # Update existing entry
            results_df.loc[existing_idx[0]] = new_row
            if verbose:
                print(f"[INFO] Updated existing entry for {model_name} + {feature_name}")
        else:
            # Add new entry
            results_df = pd.concat([results_df, pd.DataFrame([new_row])], ignore_index=True)
            if verbose:
                print(f"[INFO] Added new entry for {model_name} + {feature_name}")

        # Save results with backup
        _save_results_safely(results_df, results_file_path, verbose)

        if verbose:
            print(f"[INFO] Results saved to {results_file_path}")
            print(f"[INFO] Total entries: {len(results_df)}")

    except Exception as e:
        print(f"[ERROR] Failed to update results: {e}")
        raise


def _create_empty_results_df() -> pd.DataFrame:
    """Create an empty DataFrame with the standard results schema."""
    return pd.DataFrame(columns=[
        "Feature", "Model", "Accuracy", "MAE", "RMSE", "R2", 
        "Training Time", "Inference Time", "Last Updated"
    ])


def _save_results_safely(results_df: pd.DataFrame, file_path: str, verbose: bool = True) -> None:
    """Save results DataFrame with backup and error handling."""
    try:
        # Create backup if file exists
        if os.path.exists(file_path):
            backup_path = file_path.replace('.csv', '_backup.csv')
            pd.read_csv(file_path).to_csv(backup_path, index=False)
            if verbose:
                print(f"[INFO] Created backup at {backup_path}")
        
        # Save the updated results
        results_df.to_csv(file_path, index=False)
        
    except Exception as e:
        print(f"[ERROR] Failed to save results: {e}")
        raise


def load_results(dataset_dir: str, results_file_name: str = "model_results.csv") -> Optional[pd.DataFrame]:
    """
    Load model results from CSV file.
    
    Args:
        dataset_dir: Directory containing the results file
        results_file_name: Name of the results CSV file
        
    Returns:
        DataFrame containing results, or None if file doesn't exist
    """
    try:
        file_path = os.path.join(dataset_dir, results_file_name)
        
        if not os.path.exists(file_path):
            print(f"[WARNING] Results file not found: {file_path}")
            return None
            
        results_df = pd.read_csv(file_path)
        print(f"[INFO] Loaded {len(results_df)} results from {file_path}")
        return results_df
        
    except Exception as e:
        print(f"[ERROR] Failed to load results: {e}")
        return None