import logging
logger = logging.getLogger(__name__)

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import full_monty_features, save_feature_table, load_feature_table, process_feature_table
from dspml_pipeline.feature_extraction.handcrafted.feature_pruning import lasso_minimize_features, correlation_minimize_features, mutual_info_minimize_features
from dspml_pipeline.feature_estimation.eval_tools import evaluate_classic_models, evaluate_deep_models, validate_classical_models, classical_models_full_monty
from dspml_pipeline.results import load_results, display_feature_results
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
from dspml_pipeline.feature_extraction.learned.cnn import CNNLearnedFeatures
from dspml_pipeline.plotting_tools.data_plotting import plot_feature_reduction

from scipy import stats
import matplotlib.pyplot as plt
import yaml

def main():

    if len(sys.argv) < 2:
        raise RuntimeError("Usage: python main.py <config_file.yaml>")
    config_file = sys.argv[1]

    # Load configuration
    with open(config_file, "r") as f:
        params = yaml.safe_load(f)

    setup_logging(verbose=params['advanced']['verbose'])

    # Load data from training and validation datasets
    trainingFrameLoader = FrameLoader(dataset_dirs=params['data']['training']['dataset_dirs'],
                              target_dir=params['data']['training']['target_dir'],
                              data_log="data-log.csv",
                              label_name=params['data']['label_name'])
    validationFrameLoader = FrameLoader(dataset_dirs=params['data']['validation']['dataset_dirs'],
                              target_dir=params['data']['validation']['target_dir'],
                              data_log="data-log.csv",
                              label_name=params['data']['label_name'])

    # If new dataset, extract data. Otherwise, load from saved file.
    if params['data']['new_dataset']:
        X_train, y_train = trainingFrameLoader.extract_data()
        trainingFrameLoader.save_dataset()
        X_val, y_val = validationFrameLoader.extract_data()
        validationFrameLoader.save_dataset()
    else:
        X_train, y_train = load_dataset(dataset_dir=params['data']['training']['target_dir'])
        X_val, y_val = load_dataset(dataset_dir=params['data']['validation']['target_dir'])
     
    if params['handcrafted']['enabled']:
        
        # If a new dataset, generate handcrafted features
        if params['handcrafted']['new_features']:
            training_feature_table = full_monty_features(X=X_train, 
                                                label=y_train)
            save_feature_table(training_feature_table, 
                               params['data']['training']['target_dir'])
            validation_feature_table = full_monty_features(X=X_val, 
                                                         label=y_val)
            save_feature_table(validation_feature_table, 
                               params['data']['validation']['target_dir'])
        
        # Load features from saved feature table
        training_feature_table, training_feature_array, training_feature_names, training_labels = load_feature_table(directory=params['data']['training']['target_dir'])
        validation_feature_table, validation_feature_array, validation_feature_names, validation_labels = load_feature_table(directory=params['data']['validation']['target_dir'])

        # Train and evaluate classical models
        if params['classical']['enabled']:
            classical_models_full_monty(
                training_dir = params['data']['training']['target_dir'],
                training_labels = training_labels,
                validation_dir = params['data']['validation']['target_dir'],
                validation_labels = validation_labels,
                tune_model_params = params['classical']['tune_model_params'],
                training_features = training_feature_array,
                validation_features = validation_feature_array,
                feature_name = "Handcrafted"
            )

        # TODO: Set up training and validation for deep learning (MLP)
        # training_corr_feature_table, training_corr_features = correlation_minimize_features(feature_table=training_feature_table)
        # training_corr_feature_array, training_corr_feature_names, training_corr_labels = process_feature_table(training_corr_feature_table)

    if params['learned']['pca']['enabled']:

        n_components = params['learned']['n_features']

        # Create PCA-based features
        pca_train_tool = PCALearnedFeatures(X_train, n_components=n_components)
        pca_train_amplitude, pca_train_phase, pca_train_combined = pca_train_tool.full_monty()
        pca_val_amplitude, pca_val_phase, pca_val_combined = pca_train_tool.transform(X_val)

        # Evaluate classical models on amplitude-based PCA features
        classical_models_full_monty(
            training_dir = params['data']['training']['target_dir'],
            training_labels = y_train,
            validation_dir = params['data']['validation']['target_dir'],
            validation_labels = y_val,
            tune_model_params = params['classical']['tune_model_params'],
            training_features = pca_train_amplitude,
            validation_features = pca_val_amplitude,
            feature_name = "PCA Amplitude"
        )

        # Evaluate classical models on phase-based PCA features
        classical_models_full_monty(
            training_dir = params['data']['training']['target_dir'],
            training_labels = y_train,
            validation_dir = params['data']['validation']['target_dir'],
            validation_labels = y_val,
            tune_model_params = params['classical']['tune_model_params'],
            training_features = pca_train_phase,
            validation_features = pca_val_phase,
            feature_name = "PCA Phase"
        )

        # Evaluate classical models on combined PCA features
        classical_models_full_monty(
            training_dir = params['data']['training']['target_dir'],
            training_labels = y_train,
            validation_dir = params['data']['validation']['target_dir'],
            validation_labels = y_val,
            tune_model_params = params['classical']['tune_model_params'],
            training_features = pca_train_combined,
            validation_features = pca_val_combined,
            feature_name = "PCA Combined"
        )

        # TODO: Set up training and validation for deep learning (MLP)

    if params['learned']['kpca']['enabled']:

        n_components = params['learned']['n_features']

        kpca_train_tool = kPCALearnedFeatures(X_train, y_train, n_components=n_components)
        kpca_train_amplitude, kpca_train_phase, kpca_train_combined = kpca_train_tool.full_monty()
        kpca_val_amplitude, kpca_val_phase, kpca_val_combined = kpca_train_tool.transform(X_val)

        # Evaluate classical models on amplitude-based kPCA features
        classical_models_full_monty(
            training_dir = params['data']['training']['target_dir'],
            training_labels = y_train,
            validation_dir = params['data']['validation']['target_dir'],
            validation_labels = y_val,
            tune_model_params = params['classical']['tune_model_params'],
            training_features = kpca_train_amplitude,
            validation_features = kpca_val_amplitude,
            feature_name = "kPCA Amplitude"
        )

        # Evaluate classical models on phase-based kPCA features
        classical_models_full_monty(
            training_dir = params['data']['training']['target_dir'],
            training_labels = y_train,
            validation_dir = params['data']['validation']['target_dir'],
            validation_labels = y_val,
            tune_model_params = params['classical']['tune_model_params'],
            training_features = kpca_train_phase,
            validation_features = kpca_val_phase,
            feature_name = "kPCA Phase"
        )

        # Evaluate classical models on combined kPCA features
        classical_models_full_monty(
            training_dir = params['data']['training']['target_dir'],
            training_labels = y_train,
            validation_dir = params['data']['validation']['target_dir'],
            validation_labels = y_val,
            tune_model_params = params['classical']['tune_model_params'],
            training_features = kpca_train_combined,
            validation_features = kpca_val_combined,
            feature_name = "kPCA Combined"
        )

    if params['learned']['autoencoder']['enabled']:

        epochs = params['learned']['autoencoder']['epochs']
        batch_size = params['learned']['autoencoder']['batch_size']
        verbose = params['learned']['autoencoder']['verbose']

        # Amplitude
        X_train_amp = np.abs(X_train)
        X_val_amp = np.abs(X_val)
        autoencoder_amp = AutoencoderLearnedFeatures(X_train_amp, y_train, epochs=epochs, batch_size=batch_size, verbose=verbose)
        encoded_train_amp = autoencoder_amp.full_monty(X_train_amp)
        encoded_val_amp = autoencoder_amp.transform(X_val_amp)

        classical_models_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            tune_model_params=params['classical']['tune_model_params'],
            training_features=encoded_train_amp,
            validation_features=encoded_val_amp,
            feature_name="Autoencoder Amplitude"
        )

        # Phase
        X_train_pha = np.unwrap(np.angle(X_train))
        X_val_pha = np.unwrap(np.angle(X_val))
        autoencoder_pha = AutoencoderLearnedFeatures(X_train_pha, y_train, epochs=epochs, batch_size=batch_size, verbose=verbose)
        encoded_train_pha = autoencoder_pha.full_monty(X_train_pha)
        encoded_val_pha = autoencoder_pha.transform(X_val_pha)

        classical_models_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            tune_model_params=params['classical']['tune_model_params'],
            training_features=encoded_train_pha,
            validation_features=encoded_val_pha,
            feature_name="Autoencoder Phase"
        )

        # Combined
        X_train_com = np.concatenate((X_train_amp, X_train_pha), axis=1)
        X_val_com = np.concatenate((X_val_amp, X_val_pha), axis=1)
        autoencoder_com = AutoencoderLearnedFeatures(X_train_com, y_train, epochs=epochs, batch_size=batch_size, verbose=verbose)
        encoded_train_com = autoencoder_com.full_monty(X_train_com)
        encoded_val_com = autoencoder_com.transform(X_val_com)

        classical_models_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            tune_model_params=params['classical']['tune_model_params'],
            training_features=encoded_train_com,
            validation_features=encoded_val_com,
            feature_name="Autoencoder Combined"
        )

    if params['learned']['cnn']['enabled']:
        epochs = params['learned']['cnn']['epochs']
        verbose = params['learned']['cnn']['verbose']
        batch_size = params['learned']['cnn']['batch_size']

        # Amplitude
        X_train_amp = np.abs(X_train)
        X_val_amp = np.abs(X_val)
        cnn_amp = CNNLearnedFeatures(X_train_amp, y_train, batch_size=batch_size, epochs=epochs, verbose=verbose)
        features_train_amp = cnn_amp.full_monty(X_train_amp)
        features_val_amp = cnn_amp.transform(X_val_amp)
        feature_name_amp = "CNN Amplitude"
        classical_models_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            tune_model_params=params['classical']['tune_model_params'],
            training_features=features_train_amp,
            validation_features=features_val_amp,
            feature_name=feature_name_amp
        )

        # Phase
        X_train_pha = np.unwrap(np.angle(X_train))
        X_val_pha = np.unwrap(np.angle(X_val))
        cnn_pha = CNNLearnedFeatures(X_train_pha, y_train, batch_size=batch_size, epochs=epochs, verbose=verbose)
        features_train_pha = cnn_pha.full_monty(X_train_pha)
        features_val_pha = cnn_pha.transform(X_val_pha)
        feature_name_pha = "CNN Phase"
        classical_models_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            tune_model_params=params['classical']['tune_model_params'],
            training_features=features_train_pha,
            validation_features=features_val_pha,
            feature_name=feature_name_pha
        )

        # Combined
        X_train_com = np.concatenate((X_train_amp, X_train_pha), axis=1)
        X_val_com = np.concatenate((X_val_amp, X_val_pha), axis=1)
        cnn_com = CNNLearnedFeatures(X_train_com, y_train, batch_size=batch_size, epochs=epochs, verbose=verbose)
        features_train_com = cnn_com.full_monty(X_train_com)
        features_val_com = cnn_com.transform(X_val_com)
        feature_name_com = "CNN Combined"
        classical_models_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            tune_model_params=params['classical']['tune_model_params'],
            training_features=features_train_com,
            validation_features=features_val_com,
            feature_name=feature_name_com
        )

if __name__ == "__main__":
    main()