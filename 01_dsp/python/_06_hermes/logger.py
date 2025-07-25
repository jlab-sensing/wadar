import os
import sys
import numpy as np
import pandas as pd

def update_results(feature_name, model_name, accuracy, mae, rmse, r2, training_time, inference_time, dataset_dir, results_file="results.csv"):
    """
    Update the results CSV file with the latest model performance metrics.

    Args:
        model_name (str):           Name of the model.
        mae (float):                Mean Absolute Error of the model.
        accuracy (float):           Accuracy of the model.
        inference_time (float):     Inference time of the model.
        dataset_dir (str):          Directory where the results CSV file is stored.

    Returns:
        None
    """

    results_file = os.path.join(dataset_dir, results_file)
    if os.path.exists(results_file):
        results_df = pd.read_csv(results_file)
    else:
        results_df = pd.DataFrame(columns=["Feature", "Model", "Accuracy", "MAE", "Last Updated", "Inference Time"])

    # Check if entry for this model already exists

    existing_idx = results_df[results_df["Model"] == model_name].index.intersection(results_df[results_df["Feature"] == feature_name].index)

    new_row = {
            "Feature": feature_name,
            "Model": model_name,
            "Accuracy": accuracy,
            "MAE": mae,
            "RMSE": rmse,
            "R2": r2,
            "Training Time": training_time,
            "Last Updated": pd.Timestamp.now(),
            "Inference Time": inference_time
        }

    if len(existing_idx) > 0:
            # Update existing row
        results_df.loc[existing_idx[0]] = new_row
    else:
            # Append new row
        results_df = results_df._append(new_row, ignore_index=True)

    results_df.to_csv(results_file, index=False)