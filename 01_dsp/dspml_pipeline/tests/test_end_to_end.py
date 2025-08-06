import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import full_monty_features, save_feature_table, load_feature_table, process_feature_table
from dspml_pipeline.feature_extraction.handcrafted.feature_pruning import lasso_minimize_features, correlation_minimize_features, mutual_info_minimize_features
from dspml_pipeline.feature_estimation.eval_tools import evaluate_classic_models, evaluate_deep_models
from dspml_pipeline.results import load_results, display_feature_results
from dspml_pipeline.end_to_end_estimation.lstm import LSTMEstimator
from dspml_pipeline.results import update_results

from scipy import stats

def display_feature_importance(feature_array, feature_names, labels):
    correlations = []
    for i in range(feature_array.shape[1]):
        correlation, _ = stats.pointbiserialr(labels, feature_array[:, i])
        correlations.append((feature_names[i], correlation))

    correlations.sort(key=lambda x: abs(x[1]), reverse=True)

    for feature, corr in correlations:
        print(f"{feature}: {corr:.2f}")

if __name__ == "__main__":
    setup_logging(verbose=True)
    dataset_dirs = ["../data/wet-0-soil-compaction-dataset",
                    "../data/wet-1-soil-compaction-dataset",
                    "../data/wet-2-soil-compaction-dataset"]
    target_dir = "../data/training-dataset"
    frameLoader = FrameLoader(dataset_dirs, target_dir)
    # X, y = frameLoader.extract_data()
    # frameLoader.save_dataset()
    X, y = load_dataset(dataset_dir=target_dir)

    # ==
    
    lst = LSTMEstimator(X, y, epochs=50, batch_size=32, verbose=False)
    model, metrics = lst.full_monty()
    update_results(target_dir, "End-to-end", f"LSTM", metrics)