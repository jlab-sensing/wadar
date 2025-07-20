# This file is grossly outdated and needs to be rewritten.

# Prometheus (Προμηθεύς): The Titan who brought fire (knowledge/tools) to humanity. Represents extracting powerful insights/features from raw material.
# All image based feature extraction methods are named after Prometheus.

import matplotlib.pyplot as plt
from hydros import HydrosFrameLoader  
import numpy as np
import pywt
from eos import EosDenoising
import os
import pandas as pd

class PrometheusRadargramFeatureExtractor:
    def __init__(self):
        pass

    def build_image_dataset(self, X, output_dir="data/image_dataset"):
        N, R, T = X.shape
        if os.path.exists(output_dir):
            print(f"Warning: {output_dir} already exists. It will be overwritten.")
            for file in os.listdir(output_dir):
                file_path = os.path.join(output_dir, file)
                if os.path.isfile(file_path):
                    os.remove(file_path)
        os.makedirs(output_dir, exist_ok=True)
        image_paths = []

        for i in range(N):
            frame = X[i]
            plt.figure(figsize=(4, 3))
            plt.imshow(frame, aspect='auto', cmap='viridis')
            plt.axis('off')
            img_path = os.path.join(output_dir, f"image_{i}.png")
            plt.savefig(img_path, bbox_inches='tight', pad_inches=0)
            plt.close()
            image_paths.append(img_path)

        return image_paths
    
    def full_monty(self, X, y, output_dir="data/image_dataset"):
        image_paths = self.build_image_dataset(X, output_dir)
        df = pd.DataFrame({'image_path': image_paths, 'label': y})
        df.to_csv(os.path.join(output_dir, 'radargram_dataset.csv'), index=False)

class PrometheusWaveletFeatureExtractor:
    def __init__(self, wavelet="mexh", min_scale=1, max_scale=200, num_scales=100):
        self.wavelet = wavelet
        self.min_scale = min_scale
        self.max_scale = max_scale
        self.num_scales = num_scales

    def compute_cwt(self, frame):
        widths = np.geomspace(self.min_scale, self.max_scale, num=self.num_scales)
        coef, freqs = pywt.cwt(frame, scales=widths, wavelet=self.wavelet)
        return coef, freqs
    
    def build_image_dataset(self, X, output_dir="data/image_dataset"):
        N, R, T = X.shape
        if os.path.exists(output_dir):
            print(f"Warning: {output_dir} already exists. It will be overwritten.")
            for file in os.listdir(output_dir):
                file_path = os.path.join(output_dir, file)
                if os.path.isfile(file_path):
                    os.remove(file_path)
        os.makedirs(output_dir, exist_ok=True)
        image_paths = []

        for i in range(N):
            frame = np.median(X[i], axis=1)
            coef, _ = self.compute_cwt(frame)
            plt.figure(figsize=(4, 3))
            plt.imshow(coef, aspect='auto', cmap='viridis')
            plt.axis('off')
            img_path = os.path.join(output_dir, f"image_{i}.png")
            plt.savefig(img_path, bbox_inches='tight', pad_inches=0)
            plt.close()
            image_paths.append(img_path)

        return image_paths
    
    def full_monty(self, X, y, output_dir="data/image_dataset"):
        image_paths = self.build_image_dataset(X, output_dir)
        df = pd.DataFrame({'image_path': image_paths, 'label': y})
        df.to_csv(os.path.join(output_dir, 'cwt_dataset.csv'), index=False)
        
if __name__ == "__main__":
    # Load radar data
    dataset_dir = "data/compact-4-dry"
    hydros = HydrosFrameLoader(dataset_dir)
    X, y = hydros.load_from_dataset()
    eos = EosDenoising(n_components=1)
    # X = eos.local_pca_denoise(X)

    # prometheus = PrometheusWaveletFeatureExtractor()
    # prometheus.full_monty(X, y)
    prometheus = PrometheusRadargramFeatureExtractor()
    prometheus.full_monty(X, y)

    # test_frame = X[0]  
    # test_frame = np.median(test_frame, axis=1)

    # unique_Y, unique_Y_idx = np.unique(y, return_index=True)
    # plt.figure(figsize=(8, 2.5 * len(unique_Y)))
    # # Plot CWT coefficients for each unique label
    # for i, j in enumerate(unique_Y_idx):
    #     plt.subplot(len(unique_Y), 1, i + 1)
    #     prometheus = PrometheusFeatureExtractor()
    #     coef, freqs = compute_cwt(np.median(X[j], axis=1), wavelet="mexh", min_scale=1, max_scale=200, num_scales=100)
    #     plt.imshow(coef)
    #     plt.title(f"CWT Coefficients (Label: {unique_Y[i]:.2f})")
    #     plt.xlabel('Time')
    #     plt.ylabel('Frequency')
    #     plt.axis('off')
    # plt.tight_layout()
    # plt.show()
    # # plt.figure()