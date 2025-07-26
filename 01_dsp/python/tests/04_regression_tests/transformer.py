# transformer based regression

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _06_hermes.logger import update_results

import torch
from _04_athena.transformer import TransformerRegressionHead
from transformers import BlipProcessor, BlipForConditionalGeneration

if __name__ == "__main__":

    # Set device to GPU if available, otherwise CPU
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # Loads model from https://huggingface.co/Salesforce/blip-image-captioning-large
    processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-large")
    blip = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-large").to(device)
    output_dim = blip.vision_model.config.hidden_size

    # Loads model from Salesforce/blip-image-captioning-base (lighter version)
    # processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
    # blip = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base").to(device)
    # output_dim = blip.vision_model.config.hidden_size

    # Load and split the data
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X = np.abs(hydros.X)
    y = hydros.y

    model = TransformerRegressionHead(blip, output_dim=output_dim).to(device)
    metrics = model.full_monty_eval(X, y, processor, epochs=10, batch_size=32)

    print("Cross-validation metrics:", metrics)

    update_results("N/A", "BLIP Regression", metrics['accuracy'], metrics['mae'], metrics['rmse'], metrics['r2'], metrics['training_time'], metrics['inference_time'], dataset_dir)