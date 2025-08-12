"""Learned feature reduction using pretrained CNN."""

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import absl.logging
absl.logging.set_verbosity(absl.logging.ERROR)


import logging
logger = logging.getLogger(__name__)
logging.getLogger('absl').setLevel(logging.ERROR)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
import tensorflow as tf
from tensorflow.data import Dataset
from PIL import Image
logging.getLogger("PIL.PngImagePlugin").setLevel(logging.WARNING)
from ...parameters import RANDOM_SEED, KFOLD_SPLITS, num2label

def prepare_image_data(X: np.ndarray, y: np.ndarray = None, img_size: tuple = (160, 160)):
    """
    Prepare image data for CNN training or inference.

    Args:
        X (np.ndarray): Input data to be converted to image format.
        y (np.ndarray, optional): Labels corresponding to the input data.
        img_size (tuple): Target image size (height, width).

    Returns:
        images (np.ndarray): Array of processed images.
        y (np.ndarray, optional): Labels, if provided.
    """
    images = []
    for sample in X:
        sample_min = sample.min()
        sample_max = sample.max()
        normalized_sample = 255 * (sample - sample_min) / (sample_max - sample_min + 1e-8)
        img = Image.fromarray(normalized_sample.astype(np.uint8)).convert('RGB')
        img = img.resize(img_size)
        images.append(np.array(img))
    
    images = np.stack(images)
    
    if y is None:
        return images
    else:
        return images, y

class CNNLearnedFeatures:
    """
    Class for learned feature reduction using a pre-trained CNN (MobileNetV2).

    Attributes:
        X (np.ndarray):                 Input feature data.
        y (np.ndarray):                 Target labels.
        epochs (int):                   Number of training epochs.
        batch_size (int):               Batch size for training.
        dimensions (int):               Output feature dimension.
        img_size (tuple):               Image size for CNN input.
        output_dir (str):               Directory for saving images and labels.
        labels (list):                  List of label dictionaries.
        df (pd.DataFrame):              DataFrame of image filenames and labels.
        model (tf.keras.Model):         Keras model for feature extraction.
        verbose (bool):                 Verbosity flag for logging.
    """

    def __init__(self, X: np.ndarray, y: np.ndarray, 
                 epochs: int = 10, batch_size: int = 32, dimensions: int = 128, 
                 img_size: tuple = (160, 160), verbose: bool = False):
        """
        Initialize the class for CNN-based feature extraction.
        
        Args:
            X (np.ndarray): Input feature data.
            y (np.ndarray): Target labels.
            epochs (int, optional): Number of training epochs. Defaults to 10.
            batch_size (int, optional): Batch size for training. Defaults to 32.
            dimensions (int, optional): Output feature dimensions. Defaults to 128.
            img_size (tuple, optional): Image size as (height, width). Defaults to (160, 160).
            verbose (bool, optional): Verbosity flag for logging. Defaults to False.
        """
                        
        self.verbose = verbose
        self.epochs = epochs
        self.batch_size = batch_size
        self.dimensions = dimensions
        self.img_size = img_size
        self.output_dir = "/tmp/cnn_features"
        
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.X = X
        self.y = y
        self.labels = []
        self.df = None
        self.model = None

    def full_monty(self, X):
        """
        Performs the entire feature extraction process using the CNN.

        Args:
            X (np.ndarray): Input data to extract features from.

        Returns:
            features (np.ndarray): Extracted CNN-based features.
        """
        
        self.prepare_data()
        model, features = self.train_full()
        return features

    def preprocess(self, X: np.ndarray):
        """
        Preprocesses raw data and ensures that data isn't in IQ form.

        Args:
            X (np.ndarray): Raw input data.

        Returns:
            X (np.ndarray): Preprocessed data.
        """

        if np.iscomplex(X).any():
            logger.error("Convert the IQ data to amplitude, phase, or combine them before using as CNN input.")
        
        return X

    def prepare_data(self):
        """
        Prepares the dataset by saving images and labels to the output directory.

        Saves each sample as an image file and creates a CSV file with filenames and labels.
        """

        # Save images and labels
        for idx, sample in enumerate(self.X):
            sample_min = sample.min()           # normalize to 0-255
            sample_max = sample.max()
            normalized_sample = 255 * (sample - sample_min) / (sample_max - sample_min + 1e-8)
            img = Image.fromarray(normalized_sample.astype(np.uint8))
            label = 0  # Placeholder label, as we are extracting features
            file_name = f"sample_{idx:03d}.png"
            img.save(os.path.join(self.output_dir, file_name))
            self.labels.append({'filename': file_name, 'label': label})

        logger.info(f"Saved {len(self.X)} images to {self.output_dir}")
        self.df = pd.DataFrame(self.labels)
        self.df.to_csv(os.path.join(self.output_dir, "labels.csv"), index=False)

    def load_dataset(self):
        """
        Loads the dataset from the output directory.

        Returns:
            images (np.ndarray): Array of loaded images.
            labels (np.ndarray): Array of labels.
        """

        df = pd.read_csv(os.path.join(self.output_dir, "labels.csv"))

        def load_image(filename):
            img_path = os.path.join(self.output_dir, filename)
            img = Image.open(img_path).convert('RGB')
            img = img.resize(self.img_size)
            return np.array(img)

        images = []
        labels = []

        for idx, row in df.iterrows():
            img_array = load_image(row['filename'])
            images.append(img_array)
            labels.append(float(row['label']))  

        images = np.stack(images)
        labels = np.array(labels)

        return images, labels

    def build_model(self, train_dataset:tf.data.Dataset = None):
        """
        Builds the CNN model using a pre-trained MobileNetV2 as the base model.

        Args:
            train_dataset (tf.data.Dataset, optional): Training dataset for model summary/logging.
        """

        # Rescale pixel values 
        rescale = tf.keras.layers.Rescaling(1./127.5, offset=-1) # Rescale from [0, 255] to [-1, 1]

        # Create the base model from the pre-trained model MobileNet V2
        IMG_SHAPE = self.img_size + (3,)
        base_model = tf.keras.applications.MobileNetV2(input_shape=IMG_SHAPE,
                                                       include_top=False,
                                                       weights='imagenet')
        if self.verbose and train_dataset is not None:
            image_batch, label_batch = next(iter(train_dataset))
            feature_batch = base_model(image_batch)
            logger.info(f"Feature batch shape: {feature_batch.shape}")

        # Feature extraction
        base_model.trainable = False

        if self.verbose:
            logger.info("Base model architecture:")
            base_model.summary()

        inputs = tf.keras.Input(shape=IMG_SHAPE)
        x = rescale(inputs)
        x = base_model(x, training=False)
        x = tf.keras.layers.GlobalAveragePooling2D()(x)
        x = tf.keras.layers.Dense(self.dimensions)(x)  # projecting to (samples x self.dimensions)
        x = tf.keras.layers.Dropout(0.2)(x)
        self.model = tf.keras.Model(inputs, x)

        if self.verbose:
            self.model.summary()
            logger.info(f"Number of trainable variables: {len(self.model.trainable_variables)}")

    def train_full(self):
        """
        Trains the CNN model with the entire dataset and extracts features.

        Returns:
            model (tf.keras.Model): Trained Keras model.
            features (np.ndarray): Extracted features from the model.
        """

        images, labels = self.load_dataset()

        train_ds = Dataset.from_tensor_slices((images, labels)).batch(self.batch_size)

        AUTOTUNE = tf.data.AUTOTUNE
        train_ds = train_ds.prefetch(buffer_size=AUTOTUNE)
        train_dataset = train_ds

        self.build_model(train_dataset) # in case it was not called before

        base_learning_rate = 0.0001
        self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                           loss=tf.keras.losses.MeanSquaredError(),
                           metrics=[tf.keras.metrics.MeanSquaredError(name='mse')])
        features = self.model.predict(images)
        return self.model, features

    def transform(self, X: np.ndarray):
        """
        Applies the trained CNN model to new data to extract features.

        Args:
            X (np.ndarray): New input data of shape (samples, features).

        Returns:
            features (np.ndarray): Extracted CNN-based features.
        """

        images = prepare_image_data(X, img_size=self.img_size)
        features = self.model.predict(images)
        return features
