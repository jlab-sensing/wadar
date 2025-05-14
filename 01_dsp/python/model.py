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
    """
    Class to load, preprocess, and train a model on images for soil compaction prediction.
    The model can be used for either regression or classification tasks.
    """

    def __init__(self, data_dir, approach, img_height=227, img_width=227, batch_size=4, validation_split=0.2, seed=123):
        """
        Initialize the TENTATIVE_NAME class.

        :param data_dir: str, path to the directory containing the dataset.
        :param approach: str, either "regression" or "classification".
        :param img_height: int, height of the images.
        :param img_width: int, width of the images.
        :param batch_size: int, size of the batches for training.
        :param validation_split: float, fraction of the data to use for validation.
        :param seed: int, random seed for shuffling the dataset.
        """

        self.data_dir = pathlib.Path(data_dir)
        self.img_height = img_height
        self.img_width = img_width
        self.batch_size = batch_size
        self.validation_split = validation_split
        self.seed = seed
        self.class_names = [0, 1, 2]
        self.train_ds = None
        self.val_ds = None
        self.model = None
        self.history = None 

        self.approach = approach
        if self.approach != "regression" and self.approach != "classification":
            raise ValueError("Approach must be either 'regression' or 'classification'.")

    def load_data(self):
        """
        Load the dataset from the specified directory and preprocess the images.
        The dataset is expected to be in a CSV file with columns "filename" and "label".
        The images are resized to the specified height and width.
        """

        df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")            # This dataset is created by dataset.py such that each image has a regression label.
        labels = df["label"].tolist()
        filenames = df["filename"].tolist()

        if self.approach == "classification":
            for i in range(len(labels)):
                labels[i] = bulk_density_to_class(labels[i])                # Convert the regression labels to classification labels. 

        dataset = tf.data.Dataset.from_tensor_slices((filenames, labels))   # Dataset stored as a tuple of (filename, label)
        dataset = dataset.map(load_data)
        
        DISPLAY_IMAGES = False
        if DISPLAY_IMAGES:                                                  # This just spams all the images in the dataset. I left it here for debugging.
            for image, label in dataset:
                plt.imshow(image.numpy())
                plt.title(f"Label: {label.numpy():.2f}")
                plt.axis('off')
                plt.show()

        dataset = dataset.shuffle(buffer_size=len(df), seed=self.seed)
        dataset = dataset.shuffle(buffer_size=1000, seed=self.seed)         # Shuffle the dataset to ensure that the training and validation sets are not biased.

        val_size = int(len(df) * self.validation_split)                     # Split the dataset into training and validation sets.
        train_ds = dataset.skip(val_size)
        val_ds = dataset.take(val_size)

        train_ds = train_ds.batch(self.batch_size).prefetch(tf.data.AUTOTUNE)   # TODO: Figure out what this does.
        val_ds = val_ds.batch(self.batch_size).prefetch(tf.data.AUTOTUNE)

        self.train_ds = train_ds
        self.val_ds = val_ds

    def build_model(self):
        """
        Build the model architecture. For now, it is a simple CNN with 3 convolutional layers and 2 dense layers.
        The model is compiled with the appropriate loss function and metrics based on the approach (regression or classification).
        """

        self.model = Sequential([
            layers.Rescaling(1./255, input_shape=(self.img_height, self.img_width, 3)), # Model stolen from https://www.tensorflow.org/tutorials/images/classification
            layers.Conv2D(16, 3, padding='same', activation='relu'),
            layers.MaxPooling2D(),
            layers.Conv2D(32, 3, padding='same', activation='relu'),
            layers.MaxPooling2D(),
            layers.Conv2D(64, 3, padding='same', activation='relu'),
            layers.MaxPooling2D(),
            layers.Flatten()                                       
        ])

        if self.approach == "regression":
            self.model.add(layers.Dense(1))                         # Regression model, so only one output neuron.   
        else:
            self.model.add(layers.Dense(128, activation='relu'))
            self.model.add(layers.Dense(len(self.class_names)))        
        self.model.summary()

        if self.approach == "regression":
            self.model.compile(
                optimizer='adam',
                loss='mse',             # Mean Squared Error for regression                             
                metrics=['mae']         # Mean Absolute Error for regression
            )
        else:
            self.model.compile(
                optimizer='adam',
                loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
                metrics=['accuracy']
            )

    def train_model(self, epochs=30):
        """
        Train the model on the training dataset and validate it on the validation dataset.
        The training history is stored for later analysis.

        :param epochs: int, number of epochs to train the model.
        """

        if self.train_ds is None or self.val_ds is None:
            raise ValueError("Call load_data() before train_model().")
        if self.model is None:
            raise ValueError("Call build_model() before train_model().")

        self.history = self.model.fit(
            self.train_ds,
            validation_data=self.val_ds,
            epochs=epochs
        )

    def plot_training_results(self):
        """
        Plot the training and validation loss and accuracy (or MAE) over epochs.
        This function requires the training history to be available.
        """

        if self.history is None:
            raise ValueError("No training history found. Train the model first.")

        if self.approach == "regression":
            mae = self.history.history['mae']
            val_mae = self.history.history['val_mae']
        else:
            mae = self.history.history['accuracy']
            val_mae = self.history.history['val_accuracy']

        loss = self.history.history['loss']
        val_loss = self.history.history['val_loss']
        epochs_range = range(len(mae))

        plt.figure(figsize=(8, 8))
        plt.subplot(1, 2, 1)
        if self.approach == "regression":
            plt.plot(epochs_range, mae, label='Training MAE')
            plt.plot(epochs_range, val_mae, label='Validation MSE')
        else:
            plt.plot(epochs_range, mae, label='Training Accuracy')
            plt.plot(epochs_range, val_mae, label='Validation Accuracy')
        plt.legend(loc='lower right')
        plt.title('Training and Validation MSE')

        plt.subplot(1, 2, 2)
        plt.plot(epochs_range, loss, label='Training Loss')
        plt.plot(epochs_range, val_loss, label='Validation Loss')
        plt.legend(loc='upper right')
        plt.title('Training and Validation Loss')

        plt.show()
    
    def save_model(self, save_path):
        """
        Save the trained model to the specified path.

        :param save_path: str, path to save the model.
        """

        if self.model is None:
            raise ValueError("No model found. Train the model first.")
        self.model.save(save_path)

def load_data(filename, label, img_height=224, img_width=224, approach="regression"):
    """
    Load and preprocess the image and label.
    :param filename: str, path to the image file.
    :param label: float or int, label for the image.
    :param img_height: int, height of the image.
    :param img_width: int, width of the image.
    :param approach: str, either "regression" or "classification".
    :return: tuple, (image, label) where image is a preprocessed image and label is the corresponding label.
    """

    image = load_image(filename, img_height, img_width)
    if approach == "regression":
        return image, tf.cast(label, tf.float32)
    elif approach == "classification":
        return image, tf.cast(label, tf.int32)
    else:
        raise ValueError("Approach must be either 'regression' or 'classification'.")
    

def load_image(filename, img_height=227, img_width=227):
    """
    Load and preprocess the image.

    :param filename: str, path to the image file.
    :param img_height: int, height of the image.
    :param img_width: int, width of the image.

    :return: preprocessed image.
    """
    image = tf.io.read_file(filename)
    image = tf.image.decode_jpeg(image, channels=3) 
    image = tf.image.resize(image, (img_height, img_width))  # Resize the image to fit the model's needs
    image = image / 255.0
    return image

def bulk_density_to_class(bulk_density):
    if bulk_density <= 1.4:
        return 0                # "good"
    elif bulk_density <= 1.8:
        return 1                # "non-ideal"
    elif bulk_density < 2.0:
        return 2                # "unviable" for root growth

# ===========================================================
# TEST HARNESS
# ==========================================================
if __name__ == "__main__":
    data_dir = "data/compact-2-binary"
    approach = "classification" 

    img2compaction = Image2Compaction(data_dir, approach=approach) 
    img2compaction.load_data()
    img2compaction.build_model()
    img2compaction.train_model(epochs=30)               # Should be higher for real training?
    img2compaction.plot_training_results()
    img2compaction.save_model("model.keras")

    # Run the model
    model_path = "model.keras"
    run_model = tf.keras.models.load_model(model_path)

    df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")
    image_files = df["filename"].tolist()
    labels = df["label"].tolist()
    num_correct = 0
    for i in range(len(image_files)):
        test_image = load_image(image_files[i], img_height=224, img_width=224)
        test_image = tf.expand_dims(test_image, axis=0)  # Add batch dimension because

        prediction = run_model.predict(test_image)
        if approach == "regression":
            print(f"Image: {image_files[i]}, Predicted: {prediction[0][0]}")
        else:
            labels[i] = bulk_density_to_class(labels[i])
            prediction = tf.argmax(prediction, axis=1)
            prediction = tf.cast(prediction, tf.int32)
            print(f"Image: {image_files[i]}")
            print(f"Predicted class: {prediction[0].numpy()}")
            if prediction[0].numpy() == labels[i]:
                num_correct += 1
            print(f"Actual class: {labels[i]}")

    print(f"Accuracy: {num_correct / len(image_files) * 100:.2f}%")