import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools
import seaborn as sns
import pandas as pd
from sklearn.feature_selection import mutual_info_regression

def eval_corr(y, feature):
    """
    Evaluates the correlation between the feature and the labels.
    """
    corr = np.corrcoef(y, feature)[0, 1]
    return corr

if __name__ == "__main__":

    VIZ = False

    print("Test harness for feature pruning.")

    dataset_dir = "../../data/combined-soil-compaction-dataset"


    # If making a new feature set, 
    # hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    # X, y = hydros.X, hydros.y
    # hephaestus_features = feature_tools.FeatureTools(X)
    # feature_table = hephaestus_features.feature_full_monty(y, dataset_dir)

    # If using an existing feature set, 
    
    feature_table = pd.read_csv(f"{dataset_dir}/features.csv")

    # ==============
    # Feature selection
    # ==============

    # Mutual information tells us how much information a feature provides about the label.
    df_best, mi_scores = feature_tools.mutual_info_minimize_features(feature_table, top_n=10)
    
    print("Mutual Information of Features:")
    print(mi_scores)
    print()

    # Correlation gives us a measure of linear relationship between the feature and the label.
    corr_best, corr_scores = feature_tools.correlation_minimize_features(feature_table, top_n=10)

    print("Correlation of Features:")
    print(corr_scores)
    print()

    # Lasso regression minimizes the number of features by penalizing the absolute size of the coefficients.
    df_lasso, lasso_results = feature_tools.lasso_minimize_features(feature_table)

    print("Lasso Regression Results:")
    print(lasso_results)
    print()

    feature_tools.save_feature_table(df_best, dataset_dir)   

    # Visualize the features
    if VIZ:
        sns.set(style="whitegrid", context="paper", font_scale=1.2)

        # Mutual Information Plot
        plt.figure(figsize=(7, 4))
        sns.barplot(x='Feature', y='Mutual Information', data=mi_scores, palette='Blues_d')
        plt.title('Mutual Information of Features', fontsize=14, weight='bold')
        plt.xlabel('Feature', fontsize=12)
        plt.ylabel('Mutual Information', fontsize=12)
        plt.xticks(rotation=45, ha='right')
        plt.tight_layout()

        # Correlation Plot
        plt.figure(figsize=(7, 4))
        sns.barplot(x='Feature', y='Correlation', data=corr_scores, palette='Greens_d')
        plt.title('Correlation of Features', fontsize=14, weight='bold')
        plt.xlabel('Feature', fontsize=12)
        plt.ylabel('Correlation', fontsize=12)
        plt.xticks(rotation=45, ha='right')
        plt.tight_layout()

        # Lasso Coefficient Plot
        plt.figure(figsize=(7, 4))
        sns.barplot(x='Feature', y='Lasso Coefficients', data=lasso_results, palette='Reds_d')
        plt.title('Lasso Regression Coefficients', fontsize=14, weight='bold')
        plt.xlabel('Feature', fontsize=12)
        plt.ylabel('Lasso Coefficient', fontsize=12)
        plt.xticks(rotation=45, ha='right')
        plt.tight_layout()

        plt.show()