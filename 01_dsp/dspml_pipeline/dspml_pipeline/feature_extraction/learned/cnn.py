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

def prepare_image_data(X, y=None, img_size=(160, 160)):
    """Prepare image data for CNN training/inference."""
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
    CNN-based feature extractor using pre-trained MobileNetV2.
    Similar structure to AutoencoderLearnedFeatures but for CNN feature extraction.
    """

    def __init__(self, X, y, epochs=10, batch_size=32, dimensions=128, img_size=(160, 160), verbose=False):
        
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
        """Run the full feature extraction process."""
        self.prepare_data()
        model, features = self.train_full()
        return features

    def preprocess(self, X):
        """Preprocess input data for CNN."""
        
        if np.iscomplex(X).any():
            logger.error("Convert the IQ data to amplitude, phase, or combine them before using as CNN input.")
        
        return X

    def prepare_data(self):
        """Prepare the dataset by saving images and labels to the output directory."""
        
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
        """Load the dataset from the output directory."""
        
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

    def build_model(self, train_dataset=None):
        """Build the CNN model using a pre-trained MobileNetV2 as the base model."""
        
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
        """Train the model with the entire dataset."""
        
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
    
    def transform(self, X):
        """Extract features from input data."""
        
        images = prepare_image_data(X, img_size=self.img_size)
        features = self.model.predict(images)
        return features
