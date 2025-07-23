import numpy as np
import matplotlib.pyplot as plt
import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _04_athena.pretrained_cnn import PretrainedCNN

if __name__ == "__main__":

    dataset_dir = "../../data/combined-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X = np.abs(hydros.X)
    y = hydros.y


    output_dir = "../../data/combined-soil-compaction-dataset/images"
    trainer = PretrainedCNN(X, y, output_dir)
    model, metrics = trainer.full_monty()
    print("Cross-validation metrics:", metrics)
    # trainer.prepare_data()
    # trainer.preprocess()
    # trainer.build_model()
    # history = trainer.train(initial_epochs=30)
    # trainer.plot_history(history)
    
