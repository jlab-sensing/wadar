import os
import sys
import numpy as np
import pandas as pd

def update_results(model_name, mae, accuracy, inference_time, dataset_dir):
    results_file = os.path.join(dataset_dir, "results.csv")
    if os.path.exists(results_file):
        results_df = pd.read_csv(results_file)
    else:
        results_df = pd.DataFrame(columns=["Model", "Accuracy", "MAE", "Last Updated", "Inference Time"])

    # Check if entry for this model already exists

    existing_idx = results_df[results_df["Model"] == model_name].index

    new_row = {
            "Model": model_name,
            "Accuracy": accuracy,
            "MAE": mae,
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