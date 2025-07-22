import numpy as np
import matplotlib.pyplot as plt
import os
import sys

import tensorflow as tf
from tensorflow.data import Dataset
from _01_gaia.loader import FrameLoader
from _06_hermes.parameters import num2label
import pandas as pd
from PIL import Image

class PretrainedCNN:
    def __init__(self, dataset_dir, output_dir, img_size=(160, 160), batch_size=32):
        self.dataset_dir = dataset_dir
        self.output_dir = output_dir
        self.img_size = img_size
        self.batch_size = batch_size
        os.makedirs(self.output_dir, exist_ok=True)
        self.labels = []
        self.df = None
        self.train_dataset = None
        self.validation_dataset = None
        self.test_dataset = None
        self.model = None

    def prepare_data(self):
        hydros = FrameLoader(self.dataset_dir, new_dataset=False, ddc_flag=True)
        X = np.abs(hydros.X)
        y = hydros.y

        print("X shape:", X.shape)

        # Save images and labels
        for idx, sample in enumerate(X):
            sample_min = sample.min()           # normalize to 0-255 
            sample_max = sample.max()
            normalized_sample = 255 * (sample - sample_min) / (sample_max - sample_min + 1e-8)
            img = Image.fromarray(normalized_sample.astype(np.uint8))
            label = y[idx]
            file_name =  f"sample_{idx:03d}.png"
            img.save(os.path.join(self.output_dir, file_name))
            self.labels.append({'filename': file_name, 'label': label})

        print(f"Saved {len(X)} images to {self.output_dir}")
        self.df = pd.DataFrame(self.labels)
        self.df.to_csv(os.path.join(self.output_dir, "labels.csv"), index=False)

    def preprocess(self):
        # https://www.tensorflow.org/tutorials/images/transfer_learning
        # ====================================================
        # Data preprocessing
        # ====================================================

        df = pd.read_csv(os.path.join(self.output_dir, "labels.csv"))
        class_names = df['label'].values
        class_labels = [num2label(label) for label in class_names]

        def load_image(filename):
            img_path = os.path.join(self.output_dir, filename)
            img = Image.open(img_path).convert('RGB')
            img = img.resize(self.img_size)
            img_array = np.array(img)
            return img_array

        images = []
        labels_numeric = []

        for idx, row in df.iterrows():
            img_array = load_image(row['filename'])
            images.append(img_array)
            labels_numeric.append(int(df.loc[idx, 'label']))  # or float if regression

        images = np.stack(images)
        labels_numeric = np.array(labels_numeric)

        # Split into train/validation (e.g., 80/20 split)
        split_idx = int(0.8 * len(images))
        train_images, val_images = images[:split_idx], images[split_idx:]
        train_labels, val_labels = labels_numeric[:split_idx], labels_numeric[split_idx:]

        self.train_dataset = Dataset.from_tensor_slices((train_images, train_labels)).batch(self.batch_size)
        self.validation_dataset = Dataset.from_tensor_slices((val_images, val_labels)).batch(self.batch_size)

        val_batches = tf.data.experimental.cardinality(self.validation_dataset)
        self.test_dataset = self.validation_dataset.take(val_batches // 5)
        self.validation_dataset = self.validation_dataset.skip(val_batches // 5)

        print('Number of validation batches: %d' % tf.data.experimental.cardinality(self.validation_dataset))
        print('Number of test batches: %d' % tf.data.experimental.cardinality(self.test_dataset))

        # Configure the dataset for performance
        AUTOTUNE = tf.data.AUTOTUNE
        self.train_dataset = self.train_dataset.prefetch(buffer_size=AUTOTUNE)
        self.validation_dataset = self.validation_dataset.prefetch(buffer_size=AUTOTUNE)
        self.test_dataset = self.test_dataset.prefetch(buffer_size=AUTOTUNE)

    def build_model(self):
        # No data augmentation because radargrams are slow time x fast time, can't be flipped or 
        # rotated like images.

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
        
        image_batch, label_batch = next(iter(self.train_dataset))
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

    def train(self, initial_epochs=10):
        base_learning_rate = 0.0001
        self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                           loss=tf.keras.losses.BinaryCrossentropy(),
                           metrics=[tf.keras.metrics.BinaryAccuracy(threshold=0.5, name='accuracy')])
        
        loss0, accuracy0 = self.model.evaluate(self.validation_dataset)

        history = self.model.fit(self.train_dataset,
                                epochs=initial_epochs,
                                validation_data=self.validation_dataset)
        return history

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