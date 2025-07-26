import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
import pandas as pd

from _05_apollo import viz_tools
from _03_hephaestus import feature_tools
import tensorflow as tf
from _04_athena.cnn_models import CNN1D
from _06_hermes.logger import update_results

tf.get_logger().setLevel('ERROR')

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features
    
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"
    test_size = 0.2

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X = np.abs(hydros.X)
    y = hydros.y

    cnn_cv = CNN1D(X, y)

    model, metrics = cnn_cv.cross_validate(epochs=1000)
    print("Metrics: ", metrics)
    cnn_cv.save_model(dataset_dir)

    # cnn_cv.load_model(dataset_dir)

    metrics = cnn_cv.evaluate()

    print("CNN Model Metrics:", metrics)

    mae = metrics['mae']
    accuracy = metrics['accuracy']
    inference_time = metrics['inference_time']

    model_name = "BabyCNNRegressor"
    
    update_results(model_name, mae, accuracy, inference_time, dataset_dir)