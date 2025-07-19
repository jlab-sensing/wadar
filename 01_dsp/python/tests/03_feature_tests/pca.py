import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))# https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _05_apollo import viz_tools
from _04_athena import tree
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools
from _03_hephaestus.pca_tools import PCAProcessor
from _04_athena.simple_models import SimpleRegressor


if __name__ == "__main__":
    
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"
    test_size = 0.2
    n_components = 10

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y 

    X = np.angle(X) 
    hephaestus = PCAProcessor(X, n_components=n_components)
    features_angle = hephaestus.dimensionality_reduction()

    X = np.abs(X)
    hephaestus = PCAProcessor(X, n_components=n_components)
    features_magnitude = hephaestus.dimensionality_reduction()

    num_of_features = int(n_components / 2)
    features = np.concatenate([features_angle[:, :num_of_features], features_magnitude[:, :num_of_features]], axis=1)

    # Right now, I'm benchmarking the PCA against a random forest model
    # trained on the original features. In the future, this should be replaced
    # with the best perfmorming model(s). 


    # Using PCAProcessor to perform dimensionality reduction
    regressor = SimpleRegressor(features, y, test_size=test_size, kfold_splits=5)
    regressor.train()
    metrics_both = regressor.evaluate()

    regressor_angle = SimpleRegressor(features_angle, y, test_size=test_size, kfold_splits=5)
    regressor_angle.train()
    metrics_angle = regressor_angle.evaluate()

    regressor_magnitude = SimpleRegressor(features_magnitude, y, test_size=test_size, kfold_splits=5)
    regressor_magnitude.train()
    metrics_magnitude = regressor_magnitude.evaluate()


    # Using the manually engineered features
    feature_table, _, _, _ = feature_tools.load_feature_table(
        dataset_dir, "features.csv")
    df_best, mi_scores = feature_tools.mutual_info_minimize_features(feature_table, top_n=10)
    feature_tools.save_feature_table(df_best, dataset_dir, "features_mutual_info.csv")
    _, feature_array, feature_names, labels = feature_tools.load_feature_table(
        dataset_dir, "features_mutual_info.csv")
    
    regressor_manual = SimpleRegressor(feature_array, labels, test_size=test_size, kfold_splits=5)
    regressor_manual.train()
    metrics_manual = regressor_manual.evaluate()

    print("Manually selected features metrics:", metrics_manual)
    print()
    print("PCA features metrics:", metrics_both)
    print("PCA features (Magnitude only) metrics:", metrics_magnitude)
    print("PCA features (Angle only) metrics:", metrics_angle)