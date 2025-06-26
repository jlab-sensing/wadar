import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)
from _01_gaia.loader import FrameLoader
from _03_hephaestus import feature_tools
from _04_athena import linear_regression

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features, False to save them

    dataset_dir = "../data/compact-4-dry"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    features = feature_tools.FeatureTools(X, soil_index=200)

    features.save_features(dataset_dir, normalize=True)

    soil_compaction_targets = y
    model, metrics = linear_regression.model_linear_regression(dataset_dir, target=soil_compaction_targets)

    print("Trained model:", model)
    print("Model coefficients:", model.coef_)
    print("Model intercept:", model.intercept_)
    print("Metrics:", metrics)