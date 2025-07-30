"""
Feature Processing Utilities for Zeus Pipeline
Contains functions for processing different types of features.
"""

import numpy as np
import pandas as pd
from _03_hephaestus.feature_tools import FeatureTools
from _03_hephaestus import feature_tools
from _03_hephaestus.pca_tools import PCAProcessor
from _03_hephaestus.autoencoder import AutoencoderFeatureSelector


def process_handcrafted_features(X_train, y_train, X_val, y_val, training_dataset, validation_dataset, 
                                zeus_params, new_dataset=False):
    """
    Process handcrafted features with mutual information and correlation selection.
    
    Args:
        X_train, y_train: Training data
        X_val, y_val: Validation data
        training_dataset, validation_dataset: Dataset paths
        zeus_params: Configuration parameters
        new_dataset: Whether to create new feature tables
        
    Returns:
        Tuple of (training_features, validation_features, training_labels, validation_labels)
    """

    print("[INFO] Processing handcrafted features...")
    
    n_features = zeus_params['features']['handcrafted']['n_features']
    chosen_method = zeus_params['features']['handcrafted']['chosen_method']
    
    # Load or create features
    if new_dataset:
        print("[INFO] Creating new feature tables...")
        handcrafted_feature_engineering = FeatureTools(X_train)
        training_feature_table = handcrafted_feature_engineering.feature_full_monty(y_train, training_dataset)
        handcrafted_feature_engineering = FeatureTools(X_val)
        validation_feature_table = handcrafted_feature_engineering.feature_full_monty(y_val, validation_dataset)
    
    # Load existing feature tables
    training_feature_table, training_feature_array, training_feature_names, training_labels = feature_tools.load_feature_table(training_dataset)
    validation_feature_table, validation_feature_array, validation_feature_names, validation_labels = feature_tools.load_feature_table(validation_dataset)

    # Feature selection based on mutual information
    print(f"[INFO] Performing mutual information feature selection (top {n_features})...")
    mi_training_feature_table, _ = feature_tools.mutual_info_minimize_features(training_feature_table, top_n=n_features)
    feature_tools.save_feature_table(mi_training_feature_table, training_dataset, "mi_features.csv")
    _, mi_feature_array, mi_feature_names, mi_labels = feature_tools.load_feature_table(training_dataset, "mi_features.csv")
    
    mi_validation_feature_table = pd.concat(
        [validation_feature_table[mi_feature_names], validation_feature_table[['Label']]], axis=1
    )
    feature_tools.save_feature_table(mi_validation_feature_table, validation_dataset, "mi_features.csv")
    _, mi_validation_feature_array, mi_validation_feature_names, mi_validation_labels = feature_tools.load_feature_table(validation_dataset, "mi_features.csv")

    # Feature selection based on correlation
    print(f"[INFO] Performing correlation feature selection (top {n_features})...")
    corr_training_feature_table, _ = feature_tools.correlation_minimize_features(training_feature_table, top_n=n_features)
    feature_tools.save_feature_table(corr_training_feature_table, training_dataset, "corr_features.csv")
    _, corr_feature_array, corr_feature_names, corr_labels = feature_tools.load_feature_table(training_dataset, "corr_features.csv")
    
    # Temporarily commented out because I keep getting KeyError: "['Amplitude To Entropy Ratio 2'] not in index"
    # corr_validation_feature_table = pd.concat(
    #     [validation_feature_table[corr_feature_names], validation_feature_table[['Label']]], axis=1
    # )
    # feature_tools.save_feature_table(corr_validation_feature_table, validation_dataset, "corr_features.csv")
    # _, corr_validation_feature_array, corr_validation_feature_names, corr_validation_labels = feature_tools.load_feature_table(validation_dataset, "corr_features.csv")

    # Select features based on chosen method
    if chosen_method == 'mutual_info':
        return mi_feature_array, mi_validation_feature_array, mi_labels, mi_validation_labels
    # elif chosen_method == 'correlation':
    #     return corr_feature_array, corr_validation_feature_array, corr_labels, corr_validation_labels
    else:
        print("[WARNING] No specific feature selection method chosen, using all features.")
        return training_feature_array, validation_feature_array, training_labels, validation_labels


def process_pca_features(X_train, y_train, X_val, y_val, zeus_params):
    """
    Process PCA-based features for amplitude, phase, and combined.
    
    Args:
        X_train, y_train: Training data
        X_val, y_val: Validation data
        zeus_params: Configuration parameters
        
    Returns:
        Dictionary with amplitude, phase, and combined feature arrays
    """
    print("[INFO] Processing PCA-based features...")
    
    n_components = zeus_params['features']['pca']['n_features']
    
    # Extract amplitude and phase components
    X_train_amplitude = np.abs(X_train)
    X_val_amplitude = np.abs(X_val)
    X_train_phase = np.unwrap(np.angle(X_train))
    X_val_phase = np.unwrap(np.angle(X_val))

    # PCA processing
    print(f"[INFO] Applying PCA with {n_components} components...")
    pca_processor_amplitude = PCAProcessor(X_train_amplitude, n_components=n_components)
    pca_processor_phase = PCAProcessor(X_train_phase, n_components=n_components)

    features_amplitude_train = pca_processor_amplitude.dimensionality_reduction()
    features_amplitude_val = pca_processor_amplitude.transform(X_val_amplitude)

    features_phase_train = pca_processor_phase.dimensionality_reduction()
    features_phase_val = pca_processor_phase.transform(X_val_phase)

    top_n = len(features_amplitude_train[0]) // 2 
    features_combined_train = np.concatenate(
        [features_amplitude_train[:, :top_n], features_phase_train[:, :top_n]], axis=1
    )
    features_combined_val = np.concatenate(
        [features_amplitude_val[:, :top_n], features_phase_val[:, :top_n]], axis=1
    )

    return {
        'amplitude': {
            'train': features_amplitude_train,
            'val': features_amplitude_val,
            'labels_train': y_train,
            'labels_val': y_val
        },
        'phase': {
            'train': features_phase_train,
            'val': features_phase_val,
            'labels_train': y_train,
            'labels_val': y_val
        },
        'combined': {
            'train': features_combined_train,
            'val': features_combined_val,
            'labels_train': y_train,
            'labels_val': y_val
        }
    }


def process_autoencoder_features(X_train, y_train, X_val, y_val, zeus_params):
    """
    Process autoencoder-based features for amplitude, phase, and combined.
    
    Args:
        X_train, y_train: Training data
        X_val, y_val: Validation data
        zeus_params: Configuration parameters
        
    Returns:
        Dictionary with amplitude, phase, and combined feature arrays
    """
    print("[INFO] Processing autoencoder-based features...")
    
    n_components = zeus_params['features']['autoencoder']['n_features']
    autoencoder_params = zeus_params['features']['autoencoder']
    
    # Extract amplitude and phase components
    X_train_amplitude = np.abs(X_train)
    X_val_amplitude = np.abs(X_val)
    X_train_phase = np.unwrap(np.angle(X_train))
    X_val_phase = np.unwrap(np.angle(X_val))

    print(f"[INFO] Training autoencoder with {n_components} encoding dimensions...")
    
    # Autoencoder for amplitude features
    print("[INFO] Processing amplitude features with autoencoder...")
    if not zeus_params['features']['autoencoder']['load_model']:
        autoencoder_amplitude = AutoencoderFeatureSelector(
            X_train_amplitude, 
            encoding_dim=n_components, 
            signal_type='magnitude'
        )
    else:
        print(f"[INFO] Loading pre-trained autoencoder from {zeus_params['features']['autoencoder']['save_directory']}")
        autoencoder_amplitude = AutoencoderFeatureSelector(
            X_train_amplitude, 
            encoding_dim=n_components, 
            signal_type='magnitude',
            model_name='autoencoder_amplitude.keras',
            model_dir=zeus_params['features']['autoencoder']['save_directory'],
        )
    features_amplitude_train = autoencoder_amplitude.fit(
        epochs=autoencoder_params.get('epochs', 50),
        batch_size=autoencoder_params.get('batch_size', 32),
        patience=autoencoder_params.get('patience', 10),
        verbose=autoencoder_params.get('verbose', 0)
    )
    features_amplitude_val = autoencoder_amplitude.transform(
        autoencoder_amplitude.pre_process(X_val_amplitude, signal_type='magnitude')
    )
    
    # Autoencoder for phase features  
    print("[INFO] Processing phase features with autoencoder...")
    if not zeus_params['features']['autoencoder']['load_model']:
        autoencoder_phase = AutoencoderFeatureSelector(
            X_train_phase,
            encoding_dim=n_components,
            signal_type='phase'
        )
    else:
        print(f"[INFO] Loading pre-trained autoencoder from {zeus_params['features']['autoencoder']['save_directory']}")
        autoencoder_phase = AutoencoderFeatureSelector(
            X_train_phase,
            encoding_dim=n_components,
            signal_type='phase',
            model_name='autoencoder_phase.keras',
            model_dir=zeus_params['features']['autoencoder']['save_directory'],
        )
    features_phase_train = autoencoder_phase.fit(
        epochs=autoencoder_params.get('epochs', 50),
        batch_size=autoencoder_params.get('batch_size', 32),
        patience=autoencoder_params.get('patience', 10),
        verbose=autoencoder_params.get('verbose', 0)
    )
    features_phase_val = autoencoder_phase.transform(
        autoencoder_phase.pre_process(X_val_phase, signal_type='phase')
    )
    
    # Autoencoder for combined magnitude and phase
    print("[INFO] Processing combined features with autoencoder...")
    if not zeus_params['features']['autoencoder']['load_model']:
        autoencoder_combined = AutoencoderFeatureSelector(
            X_train, 
            encoding_dim=n_components,  # Double the encoding dimension for combined
            signal_type=None  # This will stack magnitude and phase
        )
    else:
        print(f"[INFO] Loading pre-trained autoencoder from {zeus_params['features']['autoencoder']['save_directory']}")
        autoencoder_combined = AutoencoderFeatureSelector(
            X_train,
            encoding_dim=n_components,
            signal_type=None,
            model_name='autoencoder_combined.keras',
            model_dir=zeus_params['features']['autoencoder']['save_directory'],
        )
    features_combined_train = autoencoder_combined.fit(
        epochs=autoencoder_params.get('epochs', 50),
        batch_size=autoencoder_params.get('batch_size', 32),
        patience=autoencoder_params.get('patience', 10),
        verbose=autoencoder_params.get('verbose', 0)
    )
    features_combined_val = autoencoder_combined.transform(
        autoencoder_combined.pre_process(X_val, signal_type=None)
    )
    
    # Save models if specified
    if autoencoder_params.get('save_models', False):
        save_dir = autoencoder_params.get('save_directory', './autoencoder_models')
        print(f"[INFO] Saving autoencoder models to {save_dir}...")
        import os
        os.makedirs(save_dir, exist_ok=True)
        
        autoencoder_amplitude.save_model(save_dir, 'autoencoder_amplitude.keras')
        autoencoder_phase.save_model(save_dir, 'autoencoder_phase.keras')
        autoencoder_combined.save_model(save_dir, 'autoencoder_combined.keras')

    return {
        'amplitude': {
            'train': features_amplitude_train,
            'val': features_amplitude_val,
            'labels_train': y_train,
            'labels_val': y_val,
            'model': autoencoder_amplitude
        },
        'phase': {
            'train': features_phase_train,
            'val': features_phase_val,
            'labels_train': y_train,
            'labels_val': y_val,
            'model': autoencoder_phase
        },
        'combined': {
            'train': features_combined_train,
            'val': features_combined_val,
            'labels_train': y_train,
            'labels_val': y_val,
            'model': autoencoder_combined
        }
    }
