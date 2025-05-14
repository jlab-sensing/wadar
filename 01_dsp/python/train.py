from tensorflow.keras.utils import to_categorical
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPool2D, BatchNormalization
from tensorflow.keras.layers import Activation, Dropout, Flatten, Dense
from tensorflow.keras.losses import categorical_crossentropy
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.keras.models import load_model
from sklearn.model_selection import train_test_split
from model import RegressionModel
import numpy as np
import pandas as pd
import tensorflow as tf
import cv2
from image_processing import preprocess_image
import pathlib
import matplotlib.pyplot as plt

class TrainModel:
    def __init__(self, save_path, df, epochs=10):
        self.model = RegressionModel((227, 227, 3))
        self.model = self.model.build_model()
        self.df = df
        self.save_path = save_path
        self.epochs = epochs
        self.history = None
        self.checkpoint = ModelCheckpoint(
            filepath=save_path,
            monitor='val_accuracy',
            verbose=0,
            save_best_only=True,
            save_weights_only=False,
            mode='max'
        )

    def train(self):

        Y = np.array(self.df.label.tolist())

        img_list = self.df.filename.tolist()
        X = np.array([preprocess_image(img, 227, 227) for img in img_list])
        np.save('processed_100x100_image.npy',X/255, allow_pickle=True)
        book_array = np.load('processed_100x100_image.npy',allow_pickle=True)

        X_train, X_val, Y_train, Y_val = train_test_split(X, Y, test_size=0.2, random_state=42)

        print(f"X_train shape: {X_train.shape}")
        print(f"Y_train shape: {Y_train.shape}")
        print(f"X_val shape: {X_val.shape}")
        print(f"Y_val shape: {Y_val.shape}")

        self.history = self.model.fit(X_train, Y_train, 
                    epochs = self.epochs, batch_size = 100, 
                    callbacks=[self.checkpoint], verbose=1, 
                   validation_data = (X_val, Y_val))
        
        print("Training complete.")
        print(self.history)

TEST_HARNESS = True
if TEST_HARNESS:
    data_dir = "data/compact-2-binary"
    df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")
    save_path = "model.h5"
    model = TrainModel(save_path, df)
    model.train()