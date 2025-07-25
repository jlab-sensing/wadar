import numpy as np
import matplotlib.pyplot as plt
import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _04_athena.pretrained_cnn import PretrainedCNNClassifier, PretrainedCNNRegressor, PretrainedCNNFeatureExtractor
from _06_hermes.logger import update_results

if __name__ == "__main__":

    dataset_dir = "../../data/combined-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X = np.abs(hydros.X)
    y = hydros.y


    output_dir = "../../data/combined-soil-compaction-dataset/images"

    feature_extractor = PretrainedCNNFeatureExtractor(X, y, output_dir, dimensions=2)
    _, features = feature_extractor.full_monty()
    print("Extracted features shape:", features.shape)

    plt.figure(figsize=(10, 6))
    plt.scatter(features[:, 0], features[:, 1], c=y, cmap='viridis', s=10)
    plt.colorbar(label='Labels')
    plt.title('Feature Extraction using Pretrained CNN')
    plt.xlabel('Feature 1')
    plt.ylabel('Feature 2')
    plt.show()

    # trainer = PretrainedCNNRegressor(X, y, output_dir)
    # model, metrics = trainer.full_monty(epochs=50)
    # print("Cross-validation metrics:", metrics)
    # update_results("Pretrained CNN", metrics['mae'], metrics['accuracy'], metrics['inference_time'], dataset_dir)


    # trainer = PretrainedCNNClassifier(X, y, output_dir)
    # model, metrics = trainer.full_monty()
    # print("Cross-validation metrics:", metrics)