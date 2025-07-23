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

    def __init__(self, X, y, output_dir, dimensions=128, img_size=(160, 160), batch_size=32):
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

        base_learning_rate = 0.0001
        self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                           loss=tf.keras.losses.MeanAbsoluteError(),
                           metrics=[tf.keras.metrics.MeanAbsoluteError(name='mae')])
        
        features = self.model.predict(images)

        return self.model, features

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

        # Add a regression head
        global_average_layer = tf.keras.layers.GlobalAveragePooling2D()
        feature_batch_average = global_average_layer(feature_batch)
        print(feature_batch_average.shape)

        prediction_layer = tf.keras.layers.Dense(1)
        prediction_batch = prediction_layer(feature_batch_average)
        print(prediction_batch.shape)

        inputs = tf.keras.Input(shape=IMG_SHAPE)
        x = rescale(inputs)
        x = base_model(x, training=False)
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

        base_learning_rate = 0.0001
        self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                           loss=tf.keras.losses.MeanAbsoluteError(),
                           metrics=[tf.keras.metrics.MeanAbsoluteError(name='mae')])

        loss0, mae0 = self.model.evaluate(validation_dataset)

        history = self.model.fit(train_dataset,
                                epochs=epochs,
                                validation_data=validation_dataset)
        return history


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

        base_learning_rate = 0.0001
        self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                           loss=tf.keras.losses.MeanAbsoluteError(),
                           metrics=[tf.keras.metrics.MeanAbsoluteError(name='mae')])

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
            history = self.train(train_dataset, validation_dataset, epochs=epochs)

            fold_histories.append(history)
            val_mae = history.history['val_mae'][-1]
            fold_mae.append(val_mae)

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
            'accuracy': np.mean(fold_accuracies)
        }

        return fold_histories, metrics

    def plot_history(self, history):
        acc = history.history['accuracy']
        val_acc = history.history['val_accuracy']
        loss = history.history['loss']
        val_loss = history.history['val_loss']

        plt.figure(figsize=(8, 8))
        plt.subplot(2, 1, 1)
        plt.plot(acc, label='Training Accuracy')
        plt.plot(val_acc, label='Validation Accuracy')
        plt.legend(loc='lower right')
        plt.ylabel('Accuracy')
        plt.ylim([min(plt.ylim()),1])
        plt.title('Training and Validation Accuracy')

        plt.subplot(2, 1, 2)
        plt.plot(loss, label='Training Loss')
        plt.plot(val_loss, label='Validation Loss')
        plt.legend(loc='upper right')
        plt.ylabel('Cross Entropy')
        plt.ylim([0,1.0])
        plt.title('Training and Validation Loss')
        plt.xlabel('epoch')
        plt.show()


class PretrainedCNNClassifier:
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
            labels.append(int(row['label']))  # Change to float for regression

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

        # Add a classification head
        global_average_layer = tf.keras.layers.GlobalAveragePooling2D()
        feature_batch_average = global_average_layer(feature_batch)
        print(feature_batch_average.shape)

        prediction_layer = tf.keras.layers.Dense(1, activation='sigmoid')
        prediction_batch = prediction_layer(feature_batch_average)
        print(prediction_batch.shape)

        inputs = tf.keras.Input(shape=IMG_SHAPE)
        x = rescale(inputs)
        x = base_model(x, training=False)
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

        base_learning_rate = 0.0001
        self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                           loss=tf.keras.losses.BinaryCrossentropy(),
                           metrics=[tf.keras.metrics.BinaryAccuracy(threshold=0.5, name='accuracy')])

        loss0, accuracy0 = self.model.evaluate(validation_dataset)

        history = self.model.fit(train_dataset,
                                epochs=epochs,
                                validation_data=validation_dataset)
        return history


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

        base_learning_rate = 0.0001
        self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                           loss=tf.keras.losses.BinaryCrossentropy(),
                           metrics=[tf.keras.metrics.BinaryAccuracy(threshold=0.5, name='accuracy')])

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
        fold_accuracies = []

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
            history = self.train(train_dataset, validation_dataset, epochs=epochs)

            fold_histories.append(history)
            val_acc = history.history['val_accuracy'][-1]
            fold_accuracies.append(val_acc)
            print(f"Fold {fold+1} validation accuracy: {val_acc:.4f}")

        metrics = {
            'accuracy': np.mean(fold_accuracies)
        }

        return fold_histories, metrics

    def plot_history(self, history):
        acc = history.history['accuracy']
        val_acc = history.history['val_accuracy']
        loss = history.history['loss']
        val_loss = history.history['val_loss']

        plt.figure(figsize=(8, 8))
        plt.subplot(2, 1, 1)
        plt.plot(acc, label='Training Accuracy')
        plt.plot(val_acc, label='Validation Accuracy')
        plt.legend(loc='lower right')
        plt.ylabel('Accuracy')
        plt.ylim([min(plt.ylim()),1])
        plt.title('Training and Validation Accuracy')

        plt.subplot(2, 1, 2)
        plt.plot(loss, label='Training Loss')
        plt.plot(val_loss, label='Validation Loss')
        plt.legend(loc='upper right')
        plt.ylabel('Cross Entropy')
        plt.ylim([0,1.0])
        plt.title('Training and Validation Loss')
        plt.xlabel('epoch')
        plt.show()