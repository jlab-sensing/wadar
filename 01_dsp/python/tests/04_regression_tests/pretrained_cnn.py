import numpy as np
import matplotlib.pyplot as plt
import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
sys.path.insert(0, parent_dir)

from _04_athena.pretrained_cnn import PretrainedCNN

if __name__ == "__main__":
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    output_dir = "../../data/combined-soil-compaction-dataset/images"
    trainer = PretrainedCNN(dataset_dir, output_dir)
    trainer.prepare_data()
    trainer.preprocess()
    trainer.build_model()
    history = trainer.train(initial_epochs=10)
    trainer.plot_history(history)