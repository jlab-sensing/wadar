import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
import pandas as pd

from _05_apollo import viz_tools
from _03_hephaestus import feature_tools
import tensorflow as tf
from _04_athena.cnn_models import CNN1D
from _06_hermes.logger import update_results
from _06_hermes.parameters import model_name
from _04_athena.pretrained_cnn import PretrainedCNNRegressor

# Enable and disable different model training and evaluation sections as needed.
CNN_1D = False
PRETRAINED_CNN = True

tf.get_logger().setLevel('ERROR')

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features
    
    dataset_dir = "../data/training-dataset"
    model_dir = dataset_dir + "/models"

    feature_file_name = "features.csv"
    test_size = 0.2

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X_amp = np.abs(hydros.X)
    X_phase = np.angle(hydros.X)
    X_combined = np.concatenate((X_amp, X_phase), axis=2)
    y = hydros.y

    # ============ 
    # 1D CNN 
    # ============

    epochs = 100

    if CNN_1D:

        # On amplitude only
        cnn_cv = CNN1D(X_amp, y)
        model, metrics = cnn_cv.full_monty(epochs=epochs)
        cnn_cv.save_model(model_dir, model_name="model_1d_cnn_amplitude.keras")
        print("CNN 1D Amplitude Metrics: ", metrics)
        update_results(
            feature_name="Amplitude",
            model_name="1D CNN",
            accuracy=metrics['accuracy'],
            mae=metrics['mae'],
            rmse=metrics['rmse'],
            r2=metrics['r2'],
            training_time=metrics['training_time'],
            inference_time=metrics['inference_time'],
            dataset_dir=dataset_dir,
            results_file="results-end-to-end.csv"
        )

        # On phase only
        cnn_cv = CNN1D(X_phase, y)
        model, metrics = cnn_cv.full_monty(epochs=epochs)
        cnn_cv.save_model(model_dir, model_name="model_1d_cnn_phase.keras")
        print("CNN 1D Phase Metrics: ", metrics)
        update_results(
            feature_name="Phase",
            model_name="1D CNN",
            accuracy=metrics['accuracy'],
            mae=metrics['mae'],
            rmse=metrics['rmse'],
            r2=metrics['r2'],
            training_time=metrics['training_time'],
            inference_time=metrics['inference_time'],
            dataset_dir=dataset_dir,
            results_file="results-end-to-end.csv"
        )

        # On combined amplitude and phase
        cnn_cv = CNN1D(X_combined, y)
        model, metrics = cnn_cv.full_monty(epochs=epochs)
        cnn_cv.save_model(model_dir, model_name="model_1d_cnn_combined.keras")
        print("CNN 1D Combined Metrics: ", metrics)
        update_results(
            feature_name="Combined",
            model_name="1D CNN",
            accuracy=metrics['accuracy'],
            mae=metrics['mae'],
            rmse=metrics['rmse'],
            r2=metrics['r2'],
            training_time=metrics['training_time'],
            inference_time=metrics['inference_time'],
            dataset_dir=dataset_dir,
            results_file="results-end-to-end.csv"
        )

    # ============
    # 2D CNN (transfer learning)
    # ============

    output_dir = dataset_dir + "/images"
    epochs = 30

    if PRETRAINED_CNN:
        # On amplitude only
        trainer = PretrainedCNNRegressor(X_amp, y, output_dir)
        model, metrics = trainer.full_monty(epochs=epochs)
        print("Pretrained CNN Amplitude Metrics: ", metrics)
        update_results(
            feature_name="Amplitude",
            model_name="Pretrained CNN",
            accuracy=metrics['accuracy'],
            mae=metrics['mae'],
            rmse=metrics['rmse'],
            r2=metrics['r2'],
            training_time=metrics['training_time'],
            inference_time=metrics['inference_time'],
            dataset_dir=dataset_dir,
            results_file="results-end-to-end.csv"
        )

        # On phase only
        trainer = PretrainedCNNRegressor(X_phase, y, output_dir)
        model, metrics = trainer.full_monty(epochs=epochs)
        print("Pretrained CNN Phase Metrics: ", metrics)
        update_results(
            feature_name="Phase",
            model_name="Pretrained CNN",
            accuracy=metrics['accuracy'],
            mae=metrics['mae'],
            rmse=metrics['rmse'],
            r2=metrics['r2'],
            training_time=metrics['training_time'],
            inference_time=metrics['inference_time'],
            dataset_dir=dataset_dir,
            results_file="results-end-to-end.csv"
        )

        # On combined amplitude and phase
        trainer = PretrainedCNNRegressor(X_combined, y, output_dir)
        model, metrics = trainer.full_monty(epochs=epochs)
        print("Pretrained CNN Combined Metrics: ", metrics)
        update_results(
            feature_name="Combined",
            model_name="Pretrained CNN",
            accuracy=metrics['accuracy'],
            mae=metrics['mae'],
            rmse=metrics['rmse'],
            r2=metrics['r2'],
            training_time=metrics['training_time'],
            inference_time=metrics['inference_time'],
            dataset_dir=dataset_dir,
            results_file="results-end-to-end.csv"
        )

    results = pd.read_csv(os.path.join(dataset_dir, "results-end-to-end.csv"))
    viz_tools.plot_accuracy_mae(results)