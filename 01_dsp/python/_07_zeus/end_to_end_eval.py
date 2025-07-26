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
import torch
from _04_athena.transformer import TransformerRegressionHead
from transformers import BlipProcessor, BlipForConditionalGeneration
import gc
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from _06_hermes.parameters import num2label

# Enable and disable different model training and evaluation sections as needed.
CNN_1D = True
PRETRAINED_CNN = True
TRANSFORMER = False

tf.get_logger().setLevel('ERROR')

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features
    
    dataset_dir = "../data/training-dataset"
    model_dir = dataset_dir + "/models"

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X_amp = np.abs(hydros.X)
    X_phase = np.angle(hydros.X)
    X_combined = np.concatenate((X_amp, X_phase), axis=2)
    y = hydros.y

    # ============
    # 1D CNN
    # ============

    if CNN_1D:

        # On amplitude only
        cnn_cv = CNN1D(X_amp, y)
        cnn_cv.load_model(model_dir, model_name="model_1d_cnn_amplitude.keras")
        y_pred = cnn_cv.predict(X_amp)

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None  # Not available for loaded model
        inference_time = None  # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Amplitude",
            model_name="1D CNN",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

        # On phase only
        cnn_cv = CNN1D(X_phase, y)
        cnn_cv.load_model(model_dir, model_name="model_1d_cnn_phase.keras")
        y_pred = cnn_cv.predict(X_phase)

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None  # Not available for loaded model
        inference_time = None  # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Phase",
            model_name="1D CNN",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

        # On combined amplitude and phase
        cnn_cv = CNN1D(X_combined, y)
        cnn_cv.load_model(model_dir, model_name="model_1d_cnn_combined.keras")
        y_pred = cnn_cv.predict(X_combined) 

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None  # Not available for loaded model
        inference_time = None  # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Combined",
            model_name="1D CNN",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

    # ============
    # Pretrained CNN
    # ============

    if PRETRAINED_CNN:
        output_dir = dataset_dir + "/images"
        epochs = 2

        # On amplitude only
        trainer = PretrainedCNNRegressor(X_amp, y, output_dir)
        model = trainer.load_model(model_dir + "/pretrained_cnn_amplitude.keras")
        y_pred = trainer.predict(X_amp)

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None  # Not available for loaded model
        inference_time = None  # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Amplitude",
            model_name="Pretrained CNN",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

        # On phase only
        trainer = PretrainedCNNRegressor(X_phase, y, output_dir)
        model = trainer.load_model(model_dir + "/pretrained_cnn_phase.keras")
        y_pred = trainer.predict(X_phase)

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None  # Not available for loaded model
        inference_time = None  # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Phase",
            model_name="Pretrained CNN",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

        # On combined amplitude and phase
        trainer = PretrainedCNNRegressor(X_combined, y, output_dir)
        model = trainer.load_model(model_dir + "/pretrained_cnn_combined.keras")
        y_pred = trainer.predict(X_combined)

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None  # Not available for loaded model
        inference_time = None  # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Combined",
            model_name="Pretrained CNN",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

    # ============
    # Transformer
    # ============

    device = "cuda" if torch.cuda.is_available() else "cpu"
    processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-large")
    blip = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-large").to(device)
    output_dim = blip.vision_model.config.hidden_size

    if TRANSFORMER:
        # On amplitude only
        transformer = TransformerRegressionHead(blip, output_dim=output_dim).to(device)
        transformer.load_model(model_dir + "/model_transformer_amplitude.pth")
        y_pred = transformer.predict(X_amp, processor, batch_size=32)

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None
        inference_time = None # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Amplitude",
            model_name="Transformer",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

        # On phase only
        transformer = TransformerRegressionHead(blip, output_dim=output_dim).to(device)
        transformer.load_model(model_dir + "/model_transformer_phase.pth")
        y_pred = transformer.predict(X_phase, processor, batch_size=32)

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None
        inference_time = None  # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Phase",
            model_name="Transformer",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

        # On combined amplitude and phase
        transformer = TransformerRegressionHead(blip, output_dim=output_dim).to(device)
        transformer.load_model(model_dir + "/model_transformer_combined.pth")
        y_pred = transformer.predict(X_combined, processor, batch_size=32)

        mae = mean_absolute_error(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        r2 = r2_score(y, y_pred)
        training_time = None
        inference_time = None  # Not available for loaded model
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, [num2label(label) for label in y])])
        update_results(
            feature_name="Combined",
            model_name="Transformer",
            accuracy=accuracy,
            mae=mae,
            rmse=rmse,
            r2=r2,
            training_time=training_time,
            inference_time=inference_time,
            dataset_dir=dataset_dir,
            results_file="results-end-to-end-farm.csv"
        )

    results = pd.read_csv(os.path.join(dataset_dir, "results-end-to-end-farm.csv"))
    viz_tools.plot_accuracy_mae(results)