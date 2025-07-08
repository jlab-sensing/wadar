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

def plot_feature_importance(model, feature_names):
    importances = model.feature_importances_
    indices = np.argsort(importances)[::-1]

    plt.figure(figsize=(10, 6))
    plt.barh(np.array(feature_names)[indices], importances[indices])
    plt.xlabel("Feature Importance")
    plt.title("Tree-Based Model Feature Importances")
    plt.gca().invert_yaxis()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features, False to save them

    dataset_dir = "../data/compact-4-dry"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    features = feature_tools.FeatureTools(X, soil_index=200)

    features.save_features(dataset_dir, normalize=False)

    soil_compaction_targets = y
    
    # Train Decision Tree model
    model, metrics = tree.train_decision_tree_model(
        dataset_dir, 
        target=soil_compaction_targets,
        test_size=0.2,
        random_state=42,
        max_depth=5
    )

    print("Trained Decision Tree model:", model)
    print("Metrics:", metrics)

    # plot_feature_importance(model, feature_names=list(features.feature_names))

    plot_tree(
        model,
        feature_names=features.feature_names,
        filled=True,         
        precision=3,        
        fontsize=10
    )
    plt.title("Decision Tree Structure")
    plt.show()

    # Train Random Forest model
    model_rf, metrics_rf = tree.train_random_forest_model(
        dataset_dir, 
        target=soil_compaction_targets,
        test_size=0.2,
        random_state=42,
        n_estimators=100
    )

    print("Trained Random Forest model:", model_rf)
    print("Metrics (Random Forest):", metrics_rf)

    # plot_feature_importance(model_rf, feature_names=list(features.feature_names))

    plot_tree(
        model,
        feature_names=features.feature_names,
        filled=True,         
        precision=3,        
        fontsize=10
    )
    plt.title("Decision Tree Structure")
    plt.show()