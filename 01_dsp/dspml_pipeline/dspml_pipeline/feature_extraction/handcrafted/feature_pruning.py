"""
Functions to prune features to a smaller number.
"""

import logging
logger = logging.getLogger(__name__)

import pandas as pd
import numpy as np
from sklearn.feature_selection import mutual_info_regression
from sklearn.linear_model import Lasso, LassoCV
from sklearn.preprocessing import StandardScaler


RANDOM_SEED = 42
FEATURE_COUNT = 8

def mutual_info_minimize_features(feature_table: pd.DataFrame, top_n: int = FEATURE_COUNT):
    """
    Select features using mutual information with improved efficiency.

    Parameters:
        feature_table: DataFrame containing features and labels
        top_n: Number of top features to select

    Returns:
        Tuple of (selected_features_df, mutual_info_scores_df)
    """

    logger.info(f"[INFO] Selecting top {top_n} features using mutual information...")
    
    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    labels = feature_table['Label'].values

    # Compute mutual information
    mi_scores = mutual_info_regression(feature_array, labels)
    top_feature_indices = np.argsort(mi_scores)[-top_n:][::-1]  # Descending order
    
    selected_features = [feature_names[i] for i in top_feature_indices]
    selected_mi_scores = [float(mi_scores[i]) for i in top_feature_indices]

    # Create results
    best_features_array = feature_array[:, top_feature_indices]
    df_best = pd.DataFrame(best_features_array, columns=selected_features)
    df_best['Label'] = labels

    mi_scores_df = pd.DataFrame({
        'Feature': selected_features,
        'Mutual Information': selected_mi_scores
    })

    return df_best, mi_scores_df

def correlation_minimize_features(feature_table: pd.DataFrame, top_n: int = FEATURE_COUNT):
    """
    Select features using correlation analysis with improved efficiency.

    Parameters:
        feature_table: DataFrame containing features and labels
        top_n: Number of top features to select

    Returns:
        Tuple of (selected_features_df, correlation_scores_df)
    """

    logger.info(f"Selecting top {top_n} features using correlation analysis...")
    
    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    labels = feature_table['Label'].values

    # Compute correlations efficiently
    correlations = np.array([np.corrcoef(labels, feature_array[:, i])[0, 1] 
                           for i in range(feature_array.shape[1])])
    
    # Handle NaN values
    correlations = np.nan_to_num(correlations, nan=0.0)
    
    top_feature_indices = np.argsort(np.abs(correlations))[-top_n:][::-1]
    selected_features = [feature_names[i] for i in top_feature_indices]
    selected_correlations = [float(correlations[i]) for i in top_feature_indices]

    # Create results
    best_features_array = feature_array[:, top_feature_indices]
    df_best = pd.DataFrame(best_features_array, columns=selected_features)
    df_best['Label'] = labels

    corr_scores_df = pd.DataFrame({
        'Feature': selected_features,
        'Correlation': selected_correlations
    })

    return df_best, corr_scores_df

def lasso_minimize_features(feature_table: pd.DataFrame, top_n: int = FEATURE_COUNT, alpha: float = None):
    """
    Select features using Lasso regression with L1 regularization.

    TODO: Currently this is doing a dogshit job at finding good features. Not sure why. Was heavily recommended in a reddit thread.
    
    Parameters:
        feature_table: DataFrame containing features and labels
        top_n: Number of top features to select
        alpha: Regularization strength. If None, uses cross-validation to find optimal alpha
        
    Returns:
        Tuple of (selected_features_df, lasso_scores_df)
    """
    
    logger.info(f"Selecting top {top_n} features using Lasso regression...")
    
    feature_array = feature_table.drop(columns=['Label']).values
    feature_names = feature_table.drop(columns=['Label']).columns.tolist()
    labels = feature_table['Label'].values
    
    # Standardize features for Lasso
    scaler = StandardScaler()
    feature_array_scaled = scaler.fit_transform(feature_array)
    
    # Use cross-validation to find optimal alpha if not provided
    if alpha is None:
        lasso_cv = LassoCV(cv=5, random_state=RANDOM_SEED, max_iter=20000)
        lasso_cv.fit(feature_array_scaled, labels)
        alpha = lasso_cv.alpha_
        print(f"[INFO] Optimal Lasso alpha found: {alpha:.6f}")
    
    # Fit Lasso with the selected alpha
    lasso = Lasso(alpha=alpha, random_state=RANDOM_SEED, max_iter=20000)
    lasso.fit(feature_array_scaled, labels)
    
    # Get feature coefficients and select top features by absolute coefficient value
    coefficients = np.abs(lasso.coef_)
    
    # Get indices of non-zero coefficients first, then select top_n
    non_zero_indices = np.where(coefficients > 1e-10)[0]
    
    if len(non_zero_indices) == 0:
        raise ValueError("[ERROR] No features selected by lasso_minimize_features")
    
    # Sort non-zero coefficients and select top_n
    sorted_non_zero = sorted(non_zero_indices, key=lambda i: coefficients[i], reverse=True)
    top_feature_indices = sorted_non_zero[:min(top_n, len(sorted_non_zero))]
    
    selected_features = [feature_names[i] for i in top_feature_indices]
    selected_coefficients = [float(coefficients[i]) for i in top_feature_indices]
    
    # Create results
    best_features_array = feature_array[:, top_feature_indices]
    df_best = pd.DataFrame(best_features_array, columns=selected_features)
    df_best['Label'] = labels
    
    lasso_scores_df = pd.DataFrame({
        'Feature': selected_features,
        'Lasso Coefficient': selected_coefficients
    })
    
    print(f"[INFO] Lasso selected {len(selected_features)} features out of {len(feature_names)} total")
    
    return df_best, lasso_scores_df