import numpy as np
import matplotlib.pyplot as plt
import os
import sys

import tensorflow as tf
from tensorflow.data import Dataset
from _01_gaia.loader import FrameLoader
from _06_hermes.parameters import num2label, KFOLD_SPLITS, RANDOM_SEED
import pandas as pd
from PIL import Image
import time
from sklearn.model_selection import KFold
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

class PretrainedCNNFeatureExtractor:
    """
    Pretrained CNN model for image classification. Much of the code is adapter from 
    https://www.tensorflow.org/tutorials/images/transfer_learning.

    Args:
        X (np.ndarray):                Array of images of shape (samples, height, width). 
                                       For raw radargrams, it should be of shape (samples, fast time, slow time).
        y (np.ndarray):                Array of labels of shape (samples,).
        output_dir (str):              Directory to save images and labels.
        img_size (tuple):              Size to which images will be resized (default: (160, 160)).
        batch_size (int):              Batch size for training (default: 32).
    """

    def __init__(self, X, y, output_dir, dimensions=16, img_size=(160, 160), batch_size=32):
        """
        Initialize the PretrainedCNN with images and labels.

        Args:
            X (np.ndarray):                Array of images of shape (samples, height, width).
            y (np.ndarray):                Array of labels of shape (samples,).
            output_dir (str):              Directory to save images and labels.
            img_size (tuple):              Size to which images will be resized (default: (160, 160)).
            batch_size (int):              Batch size for training (default: 32

        Returns:
            None
        """

        self.X = X
        self.output_dir = output_dir
        self.img_size = img_size
        self.dimensions = dimensions
        self.batch_size = batch_size
        os.makedirs(self.output_dir, exist_ok=True)
        self.labels = []
        self.df = None
        self.model = None
    
    def full_monty(self, epochs=10):
        """
        Run the full training and evaluation process.

        Args:
            epochs (int):                 Number of epochs for training (default: 10).

        Returns:
            model (tf.keras.Model):       The trained model.
            metrics (dict):               Dictionary containing evaluation metrics.
        """

        self.prepare_data()
        model, features = self.train_full(epochs=epochs)

        return model, features

    def prepare_data(self):
        """
        Prepare the dataset by saving images and labels to the output directory.

        Returns:
            None
        """

        # Save images and labels
        for idx, sample in enumerate(self.X):
            sample_min = sample.min()           # normalize to 0-255
            sample_max = sample.max()
            normalized_sample = 255 * (sample - sample_min) / (sample_max - sample_min + 1e-8)
            img = Image.fromarray(normalized_sample.astype(np.uint8))
            label = self.y[idx]
            file_name =  f"sample_{idx:03d}.png"
            img.save(os.path.join(self.output_dir, file_name))
            self.labels.append({'filename': file_name, 'label': label})

        print(f"Saved {len(self.X)} images to {self.output_dir}")
        self.df = pd.DataFrame(self.labels)
        self.df.to_csv(os.path.join(self.output_dir, "labels.csv"), index=False)


    def load_dataset(self):
        """
        Load the dataset from the output directory.

        Returns:
            images (np.ndarray):           Array of images of shape (samples, height, width, 3).
            labels (np.ndarray):           Array of labels of shape (samples,).
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

    def build_model(self, train_dataset=None):
        """
        Build the CNN model using a pre-trained MobileNetV2 as the base model.

        Args:
            train_dataset (tf.data.Dataset): Dataset for training (optional, used for input shape).

        Returns:
            None
        """

        # Rescale pixel values 
        rescale = tf.keras.layers.Rescaling(1./127.5, offset=-1) # Rescale from [0, 255] to [-1, 1]

        # ====================================================
        # Create the base model from the pre-trained convnets
        # ====================================================

        # Create the base model from the pre-trained model MobileNet V2
        IMG_SHAPE = self.img_size + (3,)
        base_model = tf.keras.applications.MobileNetV2(input_shape=IMG_SHAPE,
                                                       include_top=False,
                                                       weights='imagenet')
        
        image_batch, label_batch = next(iter(train_dataset))
        feature_batch = base_model(image_batch)
        print(feature_batch.shape)

        # =====================================================
        # Feature extraction
        # =====================================================

        base_model.trainable = False

        # Let's take a look at the base model architecture
        base_model.summary()

        # No head needed for feature extraction

        # global_average_layer = tf.keras.layers.GlobalAveragePooling2D()
        # feature_batch_average = global_average_layer(feature_batch)
        # print(feature_batch_average.shape)

        # prediction_layer = tf.keras.layers.Dense(1)
        # prediction_batch = prediction_layer(feature_batch_average)
        # print(prediction_batch.shape)

        inputs = tf.keras.Input(shape=IMG_SHAPE)
        x = rescale(inputs)
        x = base_model(x, training=False)
        x = tf.keras.layers.GlobalAveragePooling2D()(x)
        x = tf.keras.layers.Dense(self.dimensions)(x) # projecting to (samples x self.dimensions)
        x = tf.keras.layers.Dropout(0.2)(x)
        # outputs = prediction_layer(x)
        self.model = tf.keras.Model(inputs, x)

        self.model.summary()
        print("Number of trainable variables:", len(self.model.trainable_variables))

    def train_full(self, epochs=10):
        """
        Train the model with the entire dataset. Should only be used after cross-validation, as any validation
        done with this method will not be representative.

        Args:
            epochs (int):                 Number of epochs for training (default: 10).

        Returns:
            None
        """

        images, labels = self.load_dataset()

        train_ds = Dataset.from_tensor_slices((images, labels)).batch(self.batch_size)

        AUTOTUNE = tf.data.AUTOTUNE
        train_ds = train_ds.prefetch(buffer_size=AUTOTUNE)
        train_dataset = train_ds

        self.build_model(train_dataset) # in case it was not called before

        initial_epochs = epochs // 2
        fine_tune_epochs = epochs - initial_epochs

        self.base_model.trainable = False
        base_learning_rate = 0.0001
        self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                           loss=tf.keras.losses.MeanAbsoluteError(),
                           metrics=[tf.keras.metrics.MeanAbsoluteError(name='mae')])
        history = self.model.fit(train_dataset, epochs=initial_epochs)

        fine_tune_at = 100  # Fine-tune from this layer onwards
        self.base_model.trainable = True
        for layer in self.base_model.layers[:fine_tune_at]:
            layer.trainable = False

        self.model.compile(optimizer=tf.keras.optimizers.Adam(1e-5),
                           loss=tf.keras.losses.MeanAbsoluteError(),
                           metrics=[tf.keras.metrics.MeanAbsoluteError(name='mae')])
        history = self.model.fit(train_dataset, epochs=fine_tune_epochs)

        features = self.model.predict(images)

        return self.model, features
    
    def save_model(self, model_dir, model_name="feature_pretrained_cnn.keras"):
        os.makedirs(model_dir, exist_ok=True)
        self.model.save(os.path.join(model_dir, model_name))
        print(f"Model saved to {os.path.join(model_dir, model_name)}")

    def load_model(self, model_dir, model_name="feature_pretrained_cnn.keras"):
        self.model = tf.keras.models.load_model(os.path.join(model_dir, model_name))
        print(f"Model loaded from {os.path.join(model_dir, model_name)}")
        return self.model
    
    def estimate(self, X):
        X_scaled = self.scale_each_sample(X)
        features = self.model.predict(X_scaled)
        return features

class PretrainedCNNRegressor:
    """
    Pretrained CNN model for image classification. Much of the code is adapter from 
    https://www.tensorflow.org/tutorials/images/transfer_learning.

    Args:
        X (np.ndarray):                Array of images of shape (samples, height, width). 
                                       For raw radargrams, it should be of shape (samples, fast time, slow time).
        y (np.ndarray):                Array of labels of shape (samples,).
        output_dir (str):              Directory to save images and labels.
        img_size (tuple):              Size to which images will be resized (default: (160, 160)).
        batch_size (int):              Batch size for training (default: 32).
    """

    def __init__(self, X, y, output_dir, img_size=(160, 160), batch_size=32):
        """
        Initialize the PretrainedCNN with images and labels.

        Args:
            X (np.ndarray):                Array of images of shape (samples, height, width).
            y (np.ndarray):                Array of labels of shape (samples,).
            output_dir (str):              Directory to save images and labels.
            img_size (tuple):              Size to which images will be resized (default: (160, 160)).
            batch_size (int):              Batch size for training (default: 32

        Returns:
            None
        """

        self.X = X
        self.y = y
        self.output_dir = output_dir
        self.img_size = img_size
        self.batch_size = batch_size
        os.makedirs(self.output_dir, exist_ok=True)
        self.labels = []
        self.df = None
        self.model = None
        self.base_model = None
    
    def full_monty(self, epochs=10):
        """
        Run the full training and evaluation process.

        Args:
            epochs (int):                 Number of epochs for training (default: 10).

        Returns:
            model (tf.keras.Model):       The trained model.
            metrics (dict):               Dictionary containing evaluation metrics.
        """

        self.prepare_data()
        _, metrics = self.cross_validation(epochs=epochs)
        model = self.train_full(epochs=epochs)

        return model, metrics

    def prepare_data(self):
        """
        Prepare the dataset by saving images and labels to the output directory.

        Returns:
            None
        """

        # Save images and labels
        for idx, sample in enumerate(self.X):
            sample_min = sample.min()           # normalize to 0-255
            sample_max = sample.max()
            normalized_sample = 255 * (sample - sample_min) / (sample_max - sample_min + 1e-8)
            img = Image.fromarray(normalized_sample.astype(np.uint8))
            label = self.y[idx]
            file_name =  f"sample_{idx:03d}.png"
            img.save(os.path.join(self.output_dir, file_name))
            self.labels.append({'filename': file_name, 'label': label})

        print(f"Saved {len(self.X)} images to {self.output_dir}")
        self.df = pd.DataFrame(self.labels)
        self.df.to_csv(os.path.join(self.output_dir, "labels.csv"), index=False)


    def load_dataset(self):
        """
        Load the dataset from the output directory.

        Returns:
            images (np.ndarray):           Array of images of shape (samples, height, width, 3).
            labels (np.ndarray):           Array of labels of shape (samples,).
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

    def build_model(self, train_dataset=None):
        """
        Build the CNN model using a pre-trained MobileNetV2 as the base model.

        Args:
            train_dataset (tf.data.Dataset): Dataset for training (optional, used for input shape).

        Returns:
            None
        """

        # Rescale pixel values 
        rescale = tf.keras.layers.Rescaling(1./127.5, offset=-1) # Rescale from [0, 255] to [-1, 1]

        # ====================================================
        # Create the base model from the pre-trained convnets
        # ====================================================

        # Create the base model from the pre-trained model MobileNet V2
        IMG_SHAPE = self.img_size + (3,)
        self.base_model = tf.keras.applications.MobileNetV2(input_shape=IMG_SHAPE,
                                                       include_top=False,
                                                       weights='imagenet')
        
        image_batch, label_batch = next(iter(train_dataset))
        feature_batch = self.base_model(image_batch)
        print(feature_batch.shape)

        # =====================================================
        # Feature extraction
        # =====================================================

        self.base_model.trainable = False

        # Let's take a look at the base model architecture
        self.base_model.summary()

        # Add a regression head
        global_average_layer = tf.keras.layers.GlobalAveragePooling2D()
        feature_batch_average = global_average_layer(feature_batch)
        print(feature_batch_average.shape)

        prediction_layer = tf.keras.layers.Dense(1)
        prediction_batch = prediction_layer(feature_batch_average)
        print(prediction_batch.shape)

        inputs = tf.keras.Input(shape=IMG_SHAPE)
        x = rescale(inputs)
        x = self.base_model(x, training=False)
        x = global_average_layer(x)
        x = tf.keras.layers.Dropout(0.2)(x)
        outputs = prediction_layer(x)
        self.model = tf.keras.Model(inputs, outputs)

        self.model.summary()
        print("Number of trainable variables:", len(self.model.trainable_variables))

    def train(self, train_dataset, validation_dataset, epochs=10):
        """
        Train the CNN model.

        Args:
            train_dataset (tf.data.Dataset):        Dataset for training.
            validation_dataset (tf.data.Dataset):   Dataset for validation.
            epochs (int):                           Number of epochs for training (default: 10).

        Returns:
            history (tf.keras.callbacks.History):   Training history.
        """ 

        initial_epochs = epochs // 2
        fine_tune_epochs = epochs - initial_epochs

        self.base_model.trainable = False
        self.model.compile(optimizer=tf.keras.optimizers.Adam(1e-3),
                           loss='mae',
                           metrics=['mae'])
        history = self.model.fit(train_dataset, validation_data=validation_dataset, epochs=initial_epochs)

        fine_tune_at = 100  # Fine-tune from this layer onwards
        self.base_model.trainable = True
        for layer in self.base_model.layers[:fine_tune_at]:
            layer.trainable = False

        self.model.compile(optimizer=tf.keras.optimizers.Adam(1e-5),
                           loss='mae',
                           metrics=['mae'])
        model = self.model.fit(train_dataset, validation_data=validation_dataset, epochs=fine_tune_epochs)

        return model

    def train_full(self, epochs=10):
        """
        Train the model with the entire dataset. Should only be used after cross-validation, as any validation
        done with this method will not be representative.

        Args:
            epochs (int):                 Number of epochs for training (default: 10).

        Returns:
            None
        """

        images, labels = self.load_dataset()

        train_ds = Dataset.from_tensor_slices((images, labels)).batch(self.batch_size)

        AUTOTUNE = tf.data.AUTOTUNE
        train_ds = train_ds.prefetch(buffer_size=AUTOTUNE)
        train_dataset = train_ds

        self.build_model(train_dataset) # in case it was not called before
        model = self.train(train_dataset, train_dataset, epochs=epochs)  # Using the same dataset for training and validation

        # base_learning_rate = 0.0001
        # self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
        #                    loss=tf.keras.losses.MeanAbsoluteError(),
        #                    metrics=[tf.keras.metrics.MeanAbsoluteError(name='mae')])
        # history = self.model.fit(train_dataset, epochs=epochs)

        return self.model


    def cross_validation(self, epochs=10, kfold_splits=KFOLD_SPLITS):
        """
        Perform K-Fold cross-validation on the dataset.

        Args:
            epochs (int):                 Number of epochs for training (default: 10).
            kfold_splits (int):           Number of K-Fold splits (default: KFOLD_SPLITS).

        Returns:
            fold_histories (list):       List of training histories for each fold.
            metrics (dict):              Dictionary containing average metrics across folds.
        """

        images, labels = self.load_dataset()

        kf = KFold(n_splits=kfold_splits, shuffle=True, random_state=RANDOM_SEED)

        fold_histories = []
        fold_mae = []
        fold_inference_times = []
        fold_accuracies = []
        fold_rmse = []
        fold_r2 = []
        fold_training_times = []

        for fold, (train_idx, val_idx) in enumerate(kf.split(images)):
            print(f"\n===== Fold {fold+1}/{kfold_splits} =====")

            train_images, val_images = images[train_idx], images[val_idx]
            train_labels, val_labels = labels[train_idx], labels[val_idx]

            train_ds = Dataset.from_tensor_slices((train_images, train_labels)).batch(self.batch_size)
            val_ds = Dataset.from_tensor_slices((val_images, val_labels)).batch(self.batch_size)

            AUTOTUNE = tf.data.AUTOTUNE
            train_ds = train_ds.prefetch(buffer_size=AUTOTUNE)
            val_ds = val_ds.prefetch(buffer_size=AUTOTUNE)

            train_dataset = train_ds
            validation_dataset = val_ds

            self.build_model(train_dataset)
            time_start = time.time()
            history = self.train(train_dataset, validation_dataset, epochs=epochs)
            fold_training_times.append(time.time() - time_start)

            fold_histories.append(history)
            val_mae = history.history['val_mae'][-1]
            fold_mae.append(val_mae)
            fold_rmse.append(np.sqrt(mean_squared_error(val_labels, self.model.predict(val_images))))
            fold_r2.append(r2_score(val_labels, self.model.predict(val_images)))

            time_start = time.time()  # done separately just to measure inference time
            y_pred = self.model.predict(val_images)
            time_end = time.time()
            fold_inference_times.append(time_end - time_start)

            y_labels = [num2label(label) for label in val_labels]
            accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])
            fold_accuracies.append(accuracy)

        metrics = {
            'mae': np.mean(fold_mae),
            'inference_time': np.mean(fold_inference_times),
            'accuracy': np.mean(fold_accuracies),
            'rmse': np.mean(fold_rmse),
            'r2': np.mean(fold_r2),
            'training_time': np.mean(fold_training_times)
        }

        return fold_histories, metrics
    
    def predict(self, X):
        images = []
        for sample in X:
            sample_min = sample.min()
            sample_max = sample.max()
            normalized_sample = 255 * (sample - sample_min) / (sample_max - sample_min + 1e-8)
            img = Image.fromarray(normalized_sample.astype(np.uint8)).convert('RGB')
            img = img.resize(self.img_size)
            images.append(np.array(img))
        images = np.stack(images)
        predictions = self.model.predict(images)
        return predictions
    
    def save_model(self, model_dir, model_name="model_pretrained_cnn.keras"):
        os.makedirs(model_dir, exist_ok=True)
        self.model.save(os.path.join(model_dir, model_name))
        print(f"Model saved to {os.path.join(model_dir, model_name)}")

    def load_model(self, model_path):
        self.model = tf.keras.models.load_model(model_path)
        print(f"Model loaded from {model_path}")
        return self.model