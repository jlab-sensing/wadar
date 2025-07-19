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
from _03_hephaestus import autoencoder



if __name__ == "__main__":
    
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"
    test_size = 0.2
    encoding_dim = 10

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y 

    # Using saved model
    autoencoder_selector = autoencoder.AutoencoderFeatureSelector(X, 
                                                                  encoding_dim=encoding_dim, 
                                                                  model_name='autoencoder_model.keras',
                                                                  model_dir=dataset_dir)
    
    
    # Training a new model
    # autoencoder_selector = autoencoder.AutoencoderFeatureSelector(X, 
    #                                                               encoding_dim=encoding_dim)

    encoded_features = autoencoder_selector.fit(epochs=100, batch_size=32)
    autoencoder_selector.save_model(dataset_dir)

    # Right now, I'm benchmarking the autoencoder against a random forest model
    # trained on the original features. In the future, this should be replaced
    # with the best perfmorming model(s). 

    # Train Random Forest model on encoded features
    model, metrics_autoencoder = tree.train_random_forest(
        encoded_features, y, test_size=test_size, n_estimators=100)

    # Train Random Forest model on manually selected features
    _, feature_array, feature_names, labels = feature_tools.load_feature_table(
        dataset_dir, "feature_random_forest_monte_carlo.csv")
    
    model_rf, metrics_manual = tree.train_random_forest(
        feature_array,
        labels,
        test_size=test_size,
        n_estimators=100
    )

    print("Manually selected features metrics:", metrics_manual)
    print("Autoencoder selected features metrics:", metrics_autoencoder)