import matplotlib.pyplot as plt
import numpy as np
import os
import tensorflow as tf
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' # 3 to ignore all TensorFlow warnings
from tensorflow.keras import layers, Sequential
import pathlib

class Image2Compaction:
  def __init__(self, data_dir, img_height=227, img_width=227, batch_size=4, validation_split=0.2, seed=123):
    self.data_dir = pathlib.Path(data_dir)
    self.img_height = img_height
    self.img_width = img_width
    self.batch_size = batch_size
    self.validation_split = validation_split
    self.seed = seed
    self.class_names = []
    self.train_ds = None
    self.val_ds = None
    self.model = None
    self.history = None

  def load_data(self):
    self.train_ds = tf.keras.utils.image_dataset_from_directory(
      self.data_dir,
      validation_split=self.validation_split,
      subset="training",
      seed=self.seed,
      image_size=(self.img_height, self.img_width),
      batch_size=self.batch_size
    )

    self.val_ds = tf.keras.utils.image_dataset_from_directory(
      self.data_dir,
      validation_split=self.validation_split,
      subset="validation",
      seed=self.seed,
      image_size=(self.img_height, self.img_width),
      batch_size=self.batch_size
    )

    self.class_names = self.train_ds.class_names

    # Configure datasets for performance. No idea if this is necessary.
    AUTOTUNE = tf.data.AUTOTUNE
    self.train_ds = self.train_ds.cache().shuffle(1000).prefetch(buffer_size=AUTOTUNE)
    self.val_ds = self.val_ds.cache().prefetch(buffer_size=AUTOTUNE)

  def build_model(self):

    # Model stolen from https://www.tensorflow.org/tutorials/images/classification
    num_classes = len(self.class_names)
    self.model = Sequential([
      layers.Rescaling(1./255, input_shape=(self.img_height, self.img_width, 3)),
      layers.Conv2D(16, 3, padding='same', activation='relu'),
      layers.MaxPooling2D(),
      layers.Conv2D(32, 3, padding='same', activation='relu'),
      layers.MaxPooling2D(),
      layers.Conv2D(64, 3, padding='same', activation='relu'),
      layers.MaxPooling2D(),
      layers.Flatten(),
      layers.Dense(128, activation='relu'),
      layers.Dense(num_classes)
    ])

    self.model.compile(
      optimizer='adam',
      loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
      metrics=['accuracy']
    )

  def train_model(self, epochs=10):
    if self.model is None:
      raise ValueError("Call build_model() before train_model().")
    
    self.history = self.model.fit(
      self.train_ds,
      validation_data=self.val_ds,
      epochs=epochs
    )

  def plot_training_results(self):
    # I miss MATLAB's plot function why did I need this many lines to plot.
    if self.history is None:
      raise ValueError("No training history found. Train the model first.")

    acc = self.history.history['accuracy']
    val_acc = self.history.history['val_accuracy']
    loss = self.history.history['loss']
    val_loss = self.history.history['val_loss']
    epochs_range = range(len(acc))

    plt.figure(figsize=(8, 8))
    plt.subplot(1, 2, 1)
    plt.plot(epochs_range, acc, label='Training Accuracy')
    plt.plot(epochs_range, val_acc, label='Validation Accuracy')
    plt.legend(loc='lower right')
    plt.title('Training and Validation Accuracy')

    plt.subplot(1, 2, 2)
    plt.plot(epochs_range, loss, label='Training Loss')
    plt.plot(epochs_range, val_loss, label='Validation Loss')
    plt.legend(loc='upper right')
    plt.title('Training and Validation Loss')
    plt.show()

# ===========================================================
# TEST HARNESS
# ==========================================================
if __name__ == "__main__":
  data_dir = "data/compact-2-binary"
  img2compaction = Image2Compaction(data_dir)
  img2compaction.load_data()
  img2compaction.build_model()
  img2compaction.train_model(epochs=30)
  img2compaction.plot_training_results()