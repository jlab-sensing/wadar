import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import numpy as np

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import full_monty_features, save_feature_table, load_feature_table, process_feature_table
from dspml_pipeline.feature_extraction.handcrafted.feature_pruning import lasso_minimize_features, correlation_minimize_features, mutual_info_minimize_features
from dspml_pipeline.feature_estimation.eval_tools import evaluate_classic_models, evaluate_deep_models, validate_classical_models
from dspml_pipeline.results import load_results, display_feature_results

from scipy import stats


from dspml_pipeline.results import update_results

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

    dataset_dirs = ["../../data/wet-0-soil-compaction-dataset",
                    "../../data/wet-1-soil-compaction-dataset",
                    "../../data/wet-2-soil-compaction-dataset",
                    "../../data/field-soil-compaction-dataset"]
    target_dir = "../../data/training-dataset"
    frameLoader = FrameLoader(dataset_dirs, target_dir)
    # X_train, y_train = frameLoader.extract_data()
    # frameLoader.save_dataset()
    X_train, y_train = load_dataset(dataset_dir=target_dir)

    validation_dirs = ["../../data/field-2-soil-compaction-dataset"]
    validation_target_dir = "../../data/validation-dataset"
    frameLoader = FrameLoader(validation_dirs, validation_target_dir)
    # X_val, y_val = frameLoader.extract_data()
    # frameLoader.save_dataset()
    X_val, y_val = load_dataset(dataset_dir=validation_target_dir)

    # training_feature_table = full_monty_features(X=X_train, label=y_train)
    # save_feature_table(training_feature_table, target_dir)
    training_feature_table, training_feature_array, training_feature_names, training_labels = load_feature_table(directory=target_dir)

    # training_corr_feature_table, training_corr_features = correlation_minimize_features(feature_table=training_feature_table)
    # training_corr_feature_array, training_corr_feature_names, training_corr_labels = process_feature_table(training_corr_feature_table)

    # validation_feature_table = full_monty_features(X=X_val, label=y_val)
    # save_feature_table(validation_feature_table, validation_target_dir)
    validation_feature_table, validation_feature_array, validation_feature_names, validation_labels = load_feature_table(directory=validation_target_dir)

    # validation_corr_feature_table, validation_corr_features = correlation_minimize_features(feature_table=validation_feature_table)
    # validation_corr_feature_array, validation_corr_feature_names, validation_corr_labels = process_feature_table(validation_corr_feature_table)

    tune_model_params = True # Because tuning with a grid search is time laborious
    feature_name = "Handcrafted"


    # # ==
    
    ridgeRegressor, randomForest, gbTree, svr = evaluate_classic_models(target_dir, training_feature_array, training_labels, tune_model_params, feature_name)
    evaluate_deep_models(target_dir, validation_feature_array, validation_labels, feature_name)
    results_df = load_results(target_dir)
    # display_feature_results(feature_name, results_df)

    # Predict on the validation dataset using all trained models
    models = {
        "Ridge Regression": ridgeRegressor,
        "Random Forest": randomForest,
        "Gradient Boosting": gbTree,
        "SVR": svr
    }
    validate_classical_models(validation_target_dir, validation_feature_array, validation_labels, feature_name, models)
    results_df = load_results(validation_target_dir)
    display_feature_results(feature_name, results_df)