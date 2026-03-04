"""
File:
    main.py

Description:
    Launch file for WADAR dspml_pipeline.

Authors:
    jLab
    Eric Vetha
    nubby

Date:
    24 Feb 2026

Version:
    1.0.9
"""
import logging
logger = logging.getLogger(__name__)

import argparse
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import full_monty_features, save_feature_table, load_feature_table, process_feature_table
from dspml_pipeline.feature_extraction.handcrafted.feature_pruning import lasso_minimize_features, correlation_minimize_features, mutual_info_minimize_features
from dspml_pipeline.feature_estimation.eval_tools import classical_models_full_monty, end_to_end_model_validation, show_results_summary, deep_full_monty
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
from dspml_pipeline.feature_extraction.learned.cnn import CNNLearnedFeatures
from dspml_pipeline.end_to_end_estimation.cnn import CNNEstimator
from dspml_pipeline.end_to_end_estimation.transformer import TransformerEstimator
from dspml_pipeline.end_to_end_estimation.lstm import LSTMEstimator

from scipy import stats
import matplotlib.pyplot as plt
import yaml


def load_config(path: str) -> dict:
    """
    load_config(path)

    Load a configuration file into a return dictionary.

    Args:
        path    (str)

    Returns:
        params  (dict)
    """
    with open(path, "r") as f:
        params = yaml.safe_load(f)
    return params

def are_duplicate_examples_present(ds1: tuple, ds2: tuple) -> bool:
    """
    are_duplicate_examples_present(ds1, ds2)

    Confirm that there are no duplicated examples both within and between each dataset.

    Args:
        ds1 (tuple) First dataset.
        ds2 (tuple) Second dataset.

    Returns:
            (bool)  Are duplicates present?
    """
    dups = False
    # The dimension of each scan is (512x160), and there are many scans.
    for i, line1 in enumerate(ds1):
        for j, line2 in enumerate(ds2):
            if (len(line1) == len(line2)):
                for scan1, scan2 in zip(line1, line2):
                    if (len(scan1) == len(scan2)):
                        if tuple(scan1) == tuple(scan2):
                            print(f"Found duplicate at [{i},{j}]!")
                            dups = True
    return dups

def main(config_path: str):
    # Load training parameters from config file.
    params = load_config(path=config_path)

    # Configure logging.
    setup_logging(verbose=params['advanced']['verbose'])

    # Load data from training and validation datasets.
    trainingFrameLoader = FrameLoader(dataset_dirs=params['data']['training']['dataset_dirs'],
                              target_dir=params['data']['training']['target_dir'],
                              data_log="data-log.csv",
                              label_name=params['data']['label_name'])
    validationFrameLoader = FrameLoader(dataset_dirs=params['data']['validation']['dataset_dirs'],
                              target_dir=params['data']['validation']['target_dir'],
                              data_log="data-log.csv",
                              label_name=params['data']['label_name'])
    X_train, y_train = trainingFrameLoader.load(params['data']['new_dataset'])
    X_val, y_val = validationFrameLoader.load(params['data']['new_dataset'])

    # Verify that there are no duplicate examples in dataset.
    if (are_duplicate_examples_present(X_train, X_val)):
        print("Found duplicates! Exiting.")
        sys.exit(1)

    # TODO: Only save dataset conditionally.
    trainingFrameLoader.save_dataset()
    validationFrameLoader.save_dataset()

    """
    # If new dataset, extract data. Otherwise, load from saved file.
    if params['data']['new_dataset']:
        X_train, y_train = trainingFrameLoader.extract_data()
        # Try to load previously-processed data if none found in raw form.
        if len(X_train) > 0 and len(y_train) > 0:
            trainingFrameLoader.save_dataset()
        else:
            print(f'Loading dataset from {params["data"]["training"]["target_dir"]}.')
            X_train, y_train = load_dataset(
                    dataset_dir=params['data']['training']['target_dir'],
                    fl=trainingFrameLoader
                )
            # Exit if we still cannot find training data.
            if len(X_train) == 0 or len(y_train) == 0:
                logger.error(f'Cannot load training data for {params["data"]["training"]["target_dir"]}! Exiting.')
                sys.exit()

        # Try to load previously-processed data if none found in raw form.
        X_val, y_val = validationFrameLoader.extract_data()
        if len(X_val) > 0 and len(y_val) > 0:
            validationFrameLoader.save_dataset()
        else:
            print(f'Loading dataset from {params["data"]["validation"]["target_dir"]}.')
            X_val, y_val = load_dataset(
                    dataset_dir=params['data']['validation']['target_dir'],
                    fl=validationFrameLoader
                )
            # Exit if we still cannot find validation data.
            if len(X_val) == 0 or len(y_val) == 0:
                logger.error(f'Cannot load training data for {params["data"]["validation"]["target_dir"]}! Exiting.')
                sys.exit()
    else:
        X_train, y_train = load_dataset(
                dataset_dir=params['data']['training']['target_dir'],
                fl=trainingFrameLoader
            )
        X_val, y_val = load_dataset(
                dataset_dir=params['data']['validation']['target_dir'],
                fl=validationFrameLoader
            )
    """

    # ======== Handcrafted Features ========
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

        if params['handcrafted']['pruning_method'] == "corr":
            training_corr_feature_table, training_corr_features = correlation_minimize_features(feature_table=training_feature_table, top_n=params['deep_learning']['enabled']['top_n'])
            training_feature_array, training_feature_names, training_labels = process_feature_table(training_corr_feature_table)
        elif params['handcrafted']['pruning_method'] == "mi":
            training_corr_feature_table, training_corr_features = mutual_info_minimize_features(feature_table=training_feature_table, top_n=params['deep_learning']['enabled']['top_n'])
            training_feature_array, training_feature_names, training_labels = process_feature_table(training_corr_feature_table)
        elif params['handcrafted']['pruning_method'] == "lasso":
            training_corr_feature_table, training_corr_features = lasso_minimize_features(feature_table=training_feature_table, top_n=params['deep_learning']['enabled']['top_n'])
            training_feature_array, training_feature_names, training_labels = process_feature_table(training_corr_feature_table)

        # Use the selected features from the pruning method in the validation feature array.
        selected_feature_indices = [validation_feature_names.index(name) for name in training_feature_names]
        validation_feature_array = validation_feature_array[:, selected_feature_indices]
    
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

        # Train and evaluate classical models
        if params['deep_learning']['enabled']:
            deep_full_monty(
                training_dir=params['data']['training']['target_dir'],
                training_labels=training_labels,
                validation_dir=params['data']['validation']['target_dir'],
                validation_labels=validation_labels,
                training_features=training_feature_array,
                validation_features=validation_feature_array,
                feature_name="Handcrafted"
            )

        show_results_summary("Handcrafted", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

    # ======== PCA Features ========
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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
                training_dir=params['data']['training']['target_dir'],
                training_labels=y_train,
                validation_dir=params['data']['validation']['target_dir'],
                validation_labels=y_val,
                training_features=pca_train_amplitude,
                validation_features=pca_val_amplitude,
                feature_name="PCA Amplitude"
            )
        show_results_summary("PCA Amplitude", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
                training_dir=params['data']['training']['target_dir'],
                training_labels=y_train,
                validation_dir=params['data']['validation']['target_dir'],
                validation_labels=y_val,
                training_features=pca_train_phase,
                validation_features=pca_val_phase,
                feature_name="PCA Phase"
            )
        show_results_summary("PCA Phase", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
                training_dir=params['data']['training']['target_dir'],
                training_labels=y_train,
                validation_dir=params['data']['validation']['target_dir'],
                validation_labels=y_val,
                training_features=pca_train_combined,
                validation_features=pca_val_combined,
                feature_name="PCA Combined"
            )
        show_results_summary("PCA Combined", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

    # ======== kPCA Features ========
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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=kpca_train_amplitude,
            validation_features=kpca_val_amplitude,
            feature_name="kPCA Amplitude"
            )
        show_results_summary("kPCA Amplitude", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=kpca_train_phase,
            validation_features=kpca_val_phase,
            feature_name="kPCA Phase"
            )
        show_results_summary("kPCA Phase", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=kpca_train_combined,
            validation_features=kpca_val_combined,
            feature_name="kPCA Combined"
            )
        show_results_summary("kPCA Combined", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

    # ======== Autoencoder Features ========
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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=encoded_train_amp,
            validation_features=encoded_val_amp,
            feature_name="Autoencoder Amplitude"
            )
        show_results_summary("Autoencoder Amplitude", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=encoded_train_pha,
            validation_features=encoded_val_pha,
            feature_name="Autoencoder Phase"
            )
        show_results_summary("Autoencoder Phase", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=encoded_train_com,
            validation_features=encoded_val_com,
            feature_name="Autoencoder Combined"
            )
        show_results_summary("Autoencoder Combined", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

    # ======== CNN Features ========
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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=features_train_amp,
            validation_features=features_val_amp,
            feature_name=feature_name_amp
            )
        show_results_summary(feature_name_amp, params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=features_train_pha,
            validation_features=features_val_pha,
            feature_name=feature_name_pha
            )
        show_results_summary(feature_name_pha, params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

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
        # Train and evaluate deep learning models
        if params['deep_learning']['enabled']:
            deep_full_monty(
            training_dir=params['data']['training']['target_dir'],
            training_labels=y_train,
            validation_dir=params['data']['validation']['target_dir'],
            validation_labels=y_val,
            training_features=features_train_com,
            validation_features=features_val_com,
            feature_name=feature_name_com
            )
        show_results_summary(feature_name_com, params['data']['training']['target_dir'], params['data']['validation']['target_dir'])

    # ======== LSTM Regression ========
    model_config = params['end-to-end']['lstm']
    if model_config['enabled']:
        lstm_model = LSTMEstimator(X_train, y_train, epochs=model_config['epochs'], 
                                   batch_size=model_config['batch_size'], verbose=model_config['verbose'])
        end_to_end_model_validation(
            training_dir=params['data']['training']['target_dir'],
            validation_features=X_val,
            validation_labels=y_val,
            validation_directory=params['data']['validation']['target_dir'],
            model_class=lstm_model,
            model_name="LSTM"
        )

    # ======== CNN Regression ========
    model_config = params['end-to-end']['cnn']
    if model_config['enabled']:
        cnn = CNNEstimator(X_train, y_train, epochs=model_config['epochs'], 
                           batch_size=model_config['batch_size'], verbose=model_config['verbose'])
        end_to_end_model_validation(
            training_dir=params['data']['training']['target_dir'],
            validation_features=X_val,
            validation_labels=y_val,
            validation_directory=params['data']['validation']['target_dir'],
            model_class=cnn,
            model_name="CNN"
        )
        
    # ======== Transformer Regression ========
    model_config = params['end-to-end']['transformer']
    if model_config['enabled']:
        transformer = TransformerEstimator(X_train, y_train, 
                                           epochs=model_config['epochs'],
                                           batch_size=model_config['batch_size'],
                                           verbose=model_config['verbose'])
        end_to_end_model_validation(
            training_dir=params['data']['training']['target_dir'],
            validation_features=X_val,
            validation_labels=y_val,
            validation_directory=params['data']['validation']['target_dir'],
            model_class=transformer,
            model_name="Transformer"
        )
        
    # Display end-to-end results
    if params['end-to-end']['transformer']['enabled'] or params['end-to-end']['cnn']['enabled'] or params['end-to-end']['lstm']['enabled']:
        show_results_summary("End-to-end", params['data']['training']['target_dir'], params['data']['validation']['target_dir'])
    




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Launch training/evaluation of GOPHERS datasets.")
    parser.add_argument(
            "--config",
            "-c",
            required=True,
            type=str,
            help="Path to desired config path."
        )
    args = parser.parse_args()
    main(config_path=args.config)
