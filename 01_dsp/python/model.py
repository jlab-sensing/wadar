import matplotlib.pyplot as plt
import numpy as np
import os
import tensorflow as tf
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' # 3 to ignore all TensorFlow warnings
from tensorflow.keras import layers, Sequential
import pathlib
import pandas as pd
import cv2

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

        df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")
        df = df.sort_values(by="filename")
        labels = df["label"].tolist()
        filenames = df["filename"].tolist()

        filepaths = filenames
        dataset = tf.data.Dataset.from_tensor_slices((filepaths, labels))
        dataset = dataset.map(load_image_and_label)
        
        DISPLAY_IMAGES = False
        if DISPLAY_IMAGES:                                          # This just spams all the images in the dataset. I left it here for debugging.
            for image, label in dataset:
                plt.imshow(image.numpy())
                plt.title(f"Label: {label.numpy():.2f}")
                plt.axis('off')
                plt.show()

        dataset = dataset.shuffle(buffer_size=len(df), seed=self.seed)
        dataset = dataset.shuffle(buffer_size=1000, seed=self.seed)         # Shuffle the dataset

        val_size = int(len(df) * self.validation_split)
        train_ds = dataset.skip(val_size)
        val_ds = dataset.take(val_size)

        train_ds = train_ds.batch(self.batch_size).prefetch(tf.data.AUTOTUNE)
        val_ds = val_ds.batch(self.batch_size).prefetch(tf.data.AUTOTUNE)

        self.train_ds = train_ds
        self.val_ds = val_ds

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
            layers.Dense(1)
        ])

        self.model.compile(
            optimizer='adam',
            loss='mse',
            metrics=['mae']
        )

        self.model.summary()

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

        mae = self.history.history['mae']
        val_mae = self.history.history['val_mae']
        loss = self.history.history['loss']
        val_loss = self.history.history['val_loss']
        epochs_range = range(len(mae))

        plt.figure(figsize=(8, 8))
        plt.subplot(1, 2, 1)
        plt.plot(epochs_range, mae, label='Training MAE')
        plt.plot(epochs_range, val_mae, label='Validation MSE')
        plt.legend(loc='lower right')
        plt.title('Training and Validation MSE')

        plt.subplot(1, 2, 2)
        plt.plot(epochs_range, loss, label='Training Loss')
        plt.plot(epochs_range, val_loss, label='Validation Loss')
        plt.legend(loc='upper right')
        plt.title('Training and Validation Loss')
        plt.show()
    
    def save_model(self, save_path):
        if self.model is None:
            raise ValueError("No model found. Train the model first.")
        self.model.save(save_path)

def load_image_and_label(filename, label):
    image = tf.io.read_file(filename)
    image = tf.image.decode_jpeg(image, channels=3) 
    image = tf.image.resize(image, [227, 227])  # Use self.img_height/width if needed
    image = image / 255.0
    return image, tf.cast(label, tf.float32)

# ===========================================================
# TEST HARNESS
# ==========================================================
if __name__ == "__main__":
    data_dir = "data/compact-3"

    img2compaction = Image2Compaction(data_dir)
    img2compaction.load_data()
    img2compaction.build_model()
    img2compaction.train_model(epochs=10)
    img2compaction.plot_training_results()
    img2compaction.save_model("model.keras")

    # Run the model
    model_path = "model.keras"
    run_model = tf.keras.models.load_model(model_path)

    df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")
    image_files = df["filename"].tolist()
    for i in range(len(image_files)):
        test_image = load_image_and_label(image_files[i], 0)[0]
        test_image = tf.expand_dims(test_image, axis=0)  # Add batch dimension because

        prediction = run_model.predict(test_image)
        print(prediction)

    plt.imshow(test_image.numpy())
    plt.title(f"Label: 1.2")
    plt.axis('off')
    plt.show()