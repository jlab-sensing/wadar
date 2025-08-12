import logging
logger = logging.getLogger(__name__)

import numpy as np
from .amplitude_features import amplitude_features
from .phase_features import phase_features
from .advanced_dsp_features import advanced_dsp_features
from .segmented_features import segmented_features
from .spectral_features import spectral_features
import pandas as pd
from pathlib import Path
import sys

def full_monty_features(X: np.ndarray, label: np.ndarray):
    """
    Extract all available features for the full handcrafted feature stack.
    
    Args:
        X (np.ndarray):                 Input radar data of shape (samples, fast_time, slow_time)
        label (np.ndarray):             Target labels corresponding to each sample
        
    Returns:
        feature_table (pd.DataFrame):   Complete feature table with labels
    """
    
    logger.info("Extracting full feature set (amplitude + phase)...")
    
    # Extract various features
    amplitude_feat = amplitude_features(X)
    phase_feat = phase_features(X)
    dsp_feat = advanced_dsp_features(X)
    segmented_feat = segmented_features(X)
    spectral_feat = spectral_features(X)

    # Combine features
    feature_table = pd.concat([amplitude_feat, phase_feat, dsp_feat, segmented_feat, spectral_feat], axis=1)
    
    # Add labels
    feature_table['Label'] = label
    
    logger.info(f"Extracted {feature_table.shape[1]-1} total features ({amplitude_feat.shape[1]} amplitude + {phase_feat.shape[1]} phase)")
    
    return feature_table

def save_feature_table(feature_table: pd.DataFrame, destination: str, 
                      feature_file_name: str = 'features.csv'):
    """
    Save feature table to CSV.
    """

    try:
        destination = Path(destination)
        if not destination.exists():
            logger.error(f"Destination path does not exist at {destination}")
            
        save_path = destination / feature_file_name
        feature_table.to_csv(save_path, index=False)
        logger.info(f"Feature table saved to {save_path}")
        logger.info(f"Saved {feature_table.shape[0]} samples with {feature_table.shape[1]-1} features")
        
    except Exception as e:
        logger.error(f"Failed to save feature table: {e}")
        sys.exit(1)

def load_feature_table(directory: str, feature_file_name: str = 'features.csv'):
    """
    Load feature table from CSV.
    """

    try:
        file_path = Path(directory) / feature_file_name
        
        if not file_path.exists():
            logger.error(f"Feature file not found: {file_path}")
            sys.exit(1)
            
        feature_table = pd.read_csv(file_path)

        feature_array = feature_table.drop(columns=['Label']).values
        feature_names = feature_table.drop(columns=['Label']).columns.tolist()
        labels = feature_table['Label'].values

        logger.info(f"Loaded feature table: {feature_table.shape[0]} samples, {len(feature_names)} features")
        return feature_table, feature_array, feature_names, labels
        
    except Exception as e:
        logger.error(f"Failed to load feature table: {e}")
        sys.exit(1)

def process_feature_table(feature_table: pd.DataFrame):
    """"
    Process feature table into features, feature names, and labels.

    Returns:
        feature_array (list):   Matrix of features
        feature_names (list):   Feature names in order
        labels (list):          List of labels
    """

    try:
        feature_array = feature_table.drop(columns=['Label']).values
        feature_names = feature_table.drop(columns=['Label']).columns.tolist()
        labels = feature_table['Label'].values

        logger.info(f"Loaded feature table: {feature_table.shape[0]} samples, {len(feature_names)} features")
        return feature_array, feature_names, labels
    except Exception as e:
        logger.error(f"Failed to process feature table: {e}")
        sys.exit(1)