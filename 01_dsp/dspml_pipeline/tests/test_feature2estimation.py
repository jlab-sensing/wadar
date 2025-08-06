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
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
from dspml_pipeline.feature_extraction.learned.cnn import CNNLearnedFeatures
from dspml_pipeline.plotting_tools.data_plotting import plot_feature_reduction

from scipy import stats
import matplotlib.pyplot as plt

HANDCRAFTED = True
PCA = False
KPCA = False
AUTOENCODER = False
CNN = False

TUNE_MODEL_PARAMS = False # Because tuning with a grid search is time laborious

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

     

    if HANDCRAFTED:
        # feature_table = full_monty_features(X=X, label=y)
        # save_feature_table(feature_table, target_dir)
        feature_table, feature_array, feature_names, labels = load_feature_table(directory=target_dir)
        corr_feature_table, corr_features = correlation_minimize_features(feature_table=feature_table)
        corr_feature_array, corr_feature_names, corr_labels = process_feature_table(corr_feature_table)
        feature_name = "Handcrafted"
        evaluate_classic_models(target_dir, corr_feature_array, corr_labels, TUNE_MODEL_PARAMS, feature_name)
        evaluate_deep_models(target_dir, corr_feature_array, corr_labels, feature_name)
        results_df = load_results(target_dir)
        display_feature_results(feature_name, results_df)

    if PCA:
        n_components = 8
        pca_tool = PCALearnedFeatures(X, n_components=n_components)
        pca_amp, pca_ang, pca_combined = pca_tool.full_monty()

        # Amplitude
        feature_name_amp = "PCA Amplitude"
        evaluate_classic_models(target_dir, pca_amp, y, TUNE_MODEL_PARAMS, feature_name_amp)
        evaluate_deep_models(target_dir, pca_amp, y, feature_name_amp)
        results_df_amp = load_results(target_dir)
        display_feature_results(feature_name_amp, results_df_amp)

        # Phase
        feature_name_ang = "PCA Phase"
        evaluate_classic_models(target_dir, pca_ang, y, TUNE_MODEL_PARAMS, feature_name_ang)
        evaluate_deep_models(target_dir, pca_ang, y, feature_name_ang)
        results_df_ang = load_results(target_dir)
        display_feature_results(feature_name_ang, results_df_ang)

        # Combined
        feature_name_combined = "PCA Combined"
        evaluate_classic_models(target_dir, pca_combined, y, TUNE_MODEL_PARAMS, feature_name_combined)
        evaluate_deep_models(target_dir, pca_combined, y, feature_name_combined)
        results_df_combined = load_results(target_dir)
        display_feature_results(feature_name_combined, results_df_combined)

    if KPCA:
        n_components = 8
        kpca_tool = kPCALearnedFeatures(X, y, n_components=n_components)
        kpca_amp, kpca_ang, kpca_combined = kpca_tool.full_monty()

        # Amplitude
        feature_name_amp = "kPCA Amplitude"
        evaluate_classic_models(target_dir, kpca_amp, y, TUNE_MODEL_PARAMS, feature_name_amp)
        evaluate_deep_models(target_dir, kpca_amp, y, feature_name_amp)
        results_df_amp = load_results(target_dir)
        display_feature_results(feature_name_amp, results_df_amp)

        # Phase
        feature_name_ang = "kPCA Phase"
        evaluate_classic_models(target_dir, kpca_ang, y, TUNE_MODEL_PARAMS, feature_name_ang)
        evaluate_deep_models(target_dir, kpca_ang, y, feature_name_ang)
        results_df_ang = load_results(target_dir)
        display_feature_results(feature_name_ang, results_df_ang)

        # Combined
        feature_name_combined = "kPCA Combined"
        evaluate_classic_models(target_dir, kpca_combined, y, TUNE_MODEL_PARAMS, feature_name_combined)
        evaluate_deep_models(target_dir, kpca_combined, y, feature_name_combined)
        results_df_combined = load_results(target_dir)
        display_feature_results(feature_name_combined, results_df_combined)

    if AUTOENCODER:
        epochs = 1000
        # Amplitude
        X_amp = np.abs(X)
        autoencoder_amp = AutoencoderLearnedFeatures(X_amp, y, epochs=epochs, batch_size=256, verbose=False)
        encoded_amplitude = autoencoder_amp.full_monty(X_amp)
        feature_name_amp = "Autoencoder Amplitude"
        evaluate_classic_models(target_dir, encoded_amplitude, y, TUNE_MODEL_PARAMS, feature_name_amp)
        evaluate_deep_models(target_dir, encoded_amplitude, y, feature_name_amp)
        results_df_amp = load_results(target_dir)
        display_feature_results(feature_name_amp, results_df_amp)

        # Phase
        X_pha = np.unwrap(np.angle(X))
        autoencoder_pha = AutoencoderLearnedFeatures(X_pha, y, epochs=epochs, batch_size=256, verbose=False)
        encoded_phase = autoencoder_pha.full_monty(X_pha)
        feature_name_pha = "Autoencoder Phase"
        evaluate_classic_models(target_dir, encoded_phase, y, TUNE_MODEL_PARAMS, feature_name_pha)
        evaluate_deep_models(target_dir, encoded_phase, y, feature_name_pha)
        results_df_pha = load_results(target_dir)
        display_feature_results(feature_name_pha, results_df_pha)

        # Combined
        X_com = np.concatenate((X_amp, X_pha), axis=1)
        autoencoder_com = AutoencoderLearnedFeatures(X_com, y, epochs=epochs, batch_size=256, verbose=False)
        encoded_combined = autoencoder_com.full_monty(X_com)
        feature_name_com = "Autoencoder Combined"
        evaluate_classic_models(target_dir, encoded_combined, y, TUNE_MODEL_PARAMS, feature_name_com)
        evaluate_deep_models(target_dir, encoded_combined, y, feature_name_com)
        results_df_com = load_results(target_dir)
        display_feature_results(feature_name_com, results_df_com)

    if CNN:
        # Amplitude
        X_amp = np.abs(X)
        cnn_amp = CNNLearnedFeatures(X_amp, y, epochs=20, verbose=False)
        features_amp = cnn_amp.full_monty(X_amp)
        feature_name_amp = "CNN Amplitude"
        evaluate_classic_models(target_dir, features_amp, y, TUNE_MODEL_PARAMS, feature_name_amp)
        evaluate_deep_models(target_dir, features_amp, y, feature_name_amp)
        results_df_amp = load_results(target_dir)
        display_feature_results(feature_name_amp, results_df_amp)

        # Phase
        X_pha = np.unwrap(np.angle(X))
        cnn_pha = CNNLearnedFeatures(X_pha, y, epochs=20, verbose=False)
        features_pha = cnn_pha.full_monty(X_pha)
        feature_name_pha = "CNN Phase"
        evaluate_classic_models(target_dir, features_pha, y, TUNE_MODEL_PARAMS, feature_name_pha)
        evaluate_deep_models(target_dir, features_pha, y, feature_name_pha)
        results_df_pha = load_results(target_dir)
        display_feature_results(feature_name_pha, results_df_pha)

        # Combined
        X_com = np.concatenate((X_amp, X_pha), axis=1)
        cnn_com = CNNLearnedFeatures(X_com, y, epochs=20, verbose=False)
        features_com = cnn_com.full_monty(X_com)
        feature_name_com = "CNN Combined"
        evaluate_classic_models(target_dir, features_com, y, TUNE_MODEL_PARAMS, feature_name_com)
        evaluate_deep_models(target_dir, features_com, y, feature_name_com)
        results_df_com = load_results(target_dir)
        display_feature_results(feature_name_com, results_df_com, show_plot=True) # Plotting on the last one