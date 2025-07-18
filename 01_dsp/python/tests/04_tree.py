import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _04_athena import tree
from sklearn.tree import plot_tree
import pandas as pd

def plot_feature_importance(model, feature_names):
    importances = model.feature_importances_
    indices = np.argsort(importances)[::-1]

    plt.figure(figsize=(10, 6))
    plt.barh(np.array(feature_names)[indices], importances[indices])
    plt.xlabel("Feature Importance")
    plt.title("Tree-Based Model Feature Importances")
    plt.gca().invert_yaxis()
    plt.tight_layout()

if __name__ == "__main__": 

    VIZ = False  # Set to True to visualize features, False to save them

    dataset_dir = "../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    features = feature_tools.lasso_minimize_features(dataset_dir, X, y)    
    data = pd.read_csv(dataset_dir + '/' + feature_file_name)
    feature_names = data.drop(columns=['label']).columns.tolist()

    # Train Decision Tree model
    model, metrics = tree.train_decision_tree_model(
        dataset_dir,
        feature_file_name=feature_file_name,
        test_size=0.2,
        random_state=42,
        max_depth=5
    )

    print("Trained Decision Tree model:", model)
    print("Metrics:", metrics)


    plot_tree(
        model,
        feature_names=feature_names,
        filled=True,
        precision=3,
        fontsize=10
    )
    plt.title("Decision Tree Structure")
    plt.show()

    # Train Random Forest model
    model_rf, metrics_rf = tree.train_random_forest_model(
        dataset_dir, 
        feature_file_name=feature_file_name,
        test_size=0.2,
        random_state=42,
        n_estimators=100
    )

    print("Trained Random Forest model:", model_rf)
    print("Metrics (Random Forest):", metrics_rf)

    # plot_feature_importance(model_rf, feature_names=list(features.feature_names))

    plot_tree(
        model,
        feature_names=feature_names,
        filled=True,
        precision=3,
        fontsize=10
    )
    plt.title("Decision Tree Structure")
    plt.show()