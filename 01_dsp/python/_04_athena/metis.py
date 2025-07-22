# Thjis file is grossly outdated and needs to be rewritten.

# Metis (Μῆτις): Titaness of wisdom, prudence, and deep thought. Excellent for the intelligent and careful decision-making of an ML model.

import matplotlib.pyplot as plt
import numpy as np
import os
import tensorflow as tf
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' # 3 to ignore all TensorFlow warnings
from tensorflow.keras import layers, Sequential
import pathlib
import pandas as pd
import cv2
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from sklearn.metrics import confusion_matrix

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

        df = pd.read_csv(pathlib.Path(self.data_dir) / "radargram_dataset.csv")            # This dataset is created by dataset.py such that each image has a regression label.
        labels = df["label"].tolist()
        filenames = df["image_path"].tolist()

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
        # dataset = dataset.shuffle(buffer_size=1000, seed=self.seed)         # Shuffle the dataset to ensure that the training and validation sets are not biased.

        val_size = int(len(df) * self.validation_split)                     # Split the dataset into training and validation sets.
        train_ds = dataset.skip(val_size)
        val_ds = dataset.take(val_size)

        train_ds = train_ds.batch(self.batch_size).prefetch(tf.data.AUTOTUNE)   # TODO: Figure out what this does.
        val_ds = val_ds.batch(self.batch_size).prefetch(tf.data.AUTOTUNE)

        self.train_ds = train_ds
        self.val_ds = val_ds

    def build_model(self, epochs=100):
        """
        Build and compile the model architecture. We use transfer learning with MobileNetV2. 
        """

        # Almost everything here has been stolen from https://www.tensorflow.org/tutorials/images/transfer_learning

        data_augmentation = tf.keras.Sequential([                               # Augmenting the data to improve the model's performance.
            tf.keras.layers.RandomFlip('horizontal'),
        #   tf.keras.layers.RandomRotation(0.2),                                # Not relevant for radargrams.
        ])


        preprocess_input = tf.keras.applications.mobilenet_v2.preprocess_input  # Not used because I don't know what it does.


        IMG_SIZE = (160, 160)
        IMG_SHAPE = IMG_SIZE + (3,)
        base_model = tf.keras.applications.MobileNetV2(input_shape=IMG_SHAPE,
                                               include_top=False,           
                                               weights='imagenet')              # Using a pre-trained model.
        
        image_batch, label_batch = next(iter(self.train_ds))
        feature_batch = base_model(image_batch)                                 # Feature extraction. TODO: Feature extraction???

        base_model.trainable = False                                            # Freezing the base model we are not doing any fine-tuning.
        base_model.summary()

        if (self.approach == "classification"):

            global_average_layer = tf.keras.layers.GlobalAveragePooling2D()
            feature_batch_average = global_average_layer(feature_batch)             # Classification head.

            # prediction_layer = tf.keras.layers.Dense(1, activation='sigmoid')    
            prediction_layer = tf.keras.layers.Dense(3, activation='relu')          # relu is better according to Rezvan.
            prediction_batch = prediction_layer(feature_batch_average)              
            print(prediction_batch.shape)                                           # Not entirely sure what this tells us.

            inputs = tf.keras.Input(shape=(160, 160, 3))                            # Assembling the model here.
            x = base_model(inputs, training=False)
            x = global_average_layer(x)
            x = tf.keras.layers.Dropout(0.2)(x)
            outputs = prediction_layer(x)
            self.model = tf.keras.Model(inputs, outputs)                    

            self.model.summary()

            base_learning_rate = 0.0001                                             # Compiling the model.
            self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                loss=tf.keras.losses.SparseCategoricalCrossentropy(),
                metrics=['accuracy']
            )
            
            loss0, accuracy0 = self.model.evaluate(self.train_ds)

            print("initial loss: {:.2f}".format(loss0))
            print("initial accuracy: {:.2f}".format(accuracy0))     

            self.history = self.model.fit(self.train_ds,
                        epochs=epochs,
                        validation_data=self.val_ds)                                # Beep boop machine is learning.
        
        elif (self.approach == "regression"):
            global_average_layer = tf.keras.layers.GlobalAveragePooling2D()
            feature_batch_average = global_average_layer(feature_batch)             # Regression head.

            prediction_layer = tf.keras.layers.Dense(1, activation='relu')
            prediction_batch = prediction_layer(feature_batch_average)
            print(prediction_batch.shape)                                           # Still not entirely sure what this tells us.

            inputs = tf.keras.Input(shape=(160, 160, 3))                            # Assembling the model here.
            x = base_model(inputs, training=False)
            x = global_average_layer(x)
            x = tf.keras.layers.Dropout(0.2)(x)
            outputs = prediction_layer(x)
            self.model = tf.keras.Model(inputs, outputs)

            self.model.summary()
            base_learning_rate = 0.0001                                             # Compiling the model.

            self.model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=base_learning_rate),
                loss=tf.keras.losses.MeanAbsoluteError(),
                metrics=['mae']
            )
            loss0, mae0 = self.model.evaluate(self.train_ds)
            print("initial loss: {:.2f}".format(loss0))
            print("initial mae: {:.2f}".format(mae0))
            self.history = self.model.fit(self.train_ds,
                        epochs=epochs,
                        validation_data=self.val_ds)                                # Beep boop machine is learning.
            
        else:
            error_message = "Beep boop. What is a " + self.approach
            raise ValueError(error_message)

    def plot_training_results(self):
        """
        Plot the training and validation loss and accuracy (or MAE) over epochs.
        This function requires the training history to be available.
        """

        if self.history is None:
            raise ValueError("No training history found. Train the model first.")
        
        if self.approach == "classification":

            acc = self.history.history['accuracy']
            val_acc = self.history.history['val_accuracy']

            loss = self.history.history['loss']
            val_loss = self.history.history['val_loss']

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

            plt.show()
        
        elif self.approach == "regression":

            mae = self.history.history['mae']
            val_mae = self.history.history['val_mae']

            loss = self.history.history['loss']
            val_loss = self.history.history['val_loss']

            plt.figure(figsize=(8, 8))
            plt.subplot(2, 1, 1)
            plt.plot(mae, label='Training MAE')
            plt.plot(val_mae, label='Validation MAE')
            plt.legend(loc='lower right')
            plt.ylabel('MAE')
            plt.ylim([min(plt.ylim()),1])
            plt.title('Training and Validation MAE')

            plt.subplot(2, 1, 2)
            plt.plot(loss, label='Training Loss')
            plt.plot(val_loss, label='Validation Loss')
            plt.legend(loc='upper right')
            plt.ylabel('MAE Loss')
            plt.ylim([0,1.0])
            plt.title('Training and Validation Loss')
            plt.xlabel('epoch')
            plt.show()
        
        else:
            error_message = "Beep boop. What is a " + self.approach
            raise ValueError(error_message)
        
    def save_model(self, save_path):
        """
        Save the trained model to the specified path.

        :param save_path: str, path to save the model.
        """

        if self.model is None:
            raise ValueError("No model found. Train the model first.")
        self.model.save(save_path)

    def load_model(self, load_path):
        """
        Load a model from the specified path.

        :param load_path: str, path to load the model from.
        """

        self.model = tf.keras.models.load_model(load_path)

    def predict(self, image):
        """
        Predict the label for a given image using the trained model.

        :param image: preprocessed image.
        :return: predicted label.
        """

        if self.model is None:
            raise ValueError("No model found. Train the model first.")
        
        image = tf.expand_dims(image, axis=0)
        prediction = run_model.predict(image)

        return prediction[0][0]
    
# ===========================================================
# DATA LOADING FUNCTIONS
# ==========================================================

def load_data(filename, label, img_height=160, img_width=160, approach="regression"):
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
    

def load_image(filename, img_height=160, img_width=160):
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
    # image = image / 255.0
    # image = preprocess_input(image)
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
    data_dir = "data/image_dataset" 
    approach = "regression"  # or "regression"

    img2compaction = Image2Compaction(data_dir, approach=approach) 
    img2compaction.load_data()
    img2compaction.build_model()
    img2compaction.plot_training_results()
    img2compaction.save_model("model.keras")

    # # Run the model
    # model_path = "model.keras"
    # img2compaction = Image2Compaction(data_dir, approach=approach)
    # img2compaction.load_model(model_path)

    # # Loading test set (taken in field)
    # data_dir = "data/test_set"
    # df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")
    # image_files = df["filename"].tolist()
    # labels = df["label"].tolist()
    # predictions = []

    # for i in range(len(image_files)):
    #     image = load_image(image_files[i], img_height=160, img_width=160)
    #     image = tf.expand_dims(image, axis=0)
    #     prediction = img2compaction.model.predict(image)
    #     print(f"Image: {image_files[i]}, Prediction: {prediction[0][0]:.2f}, Actual: {labels[i]:.2f}")
    #     predictions.append(prediction[0][0])
    
    # goodActual = []
    # goodPred = []
    # goodCorrect = 0
    # goodIncorrect = 0
    # nonIdealActual = []
    # nonIdealPred = []
    # nonIdealCorrect = 0
    # nonIdealIncorrect = 0
    # unviableActual = []
    # unviablePred = []
    # unviableCorrect = 0
    # unviableIncorrect = 0
    # for i in range(len(image_files)):
    #     if labels[i] <= 1.4:
    #         goodActual.append(labels[i])
    #         goodPred.append(predictions[i])
    #         if predictions[i] <= 1.4:
    #             goodCorrect += 1
    #         else:
    #             goodIncorrect += 1
    #     elif labels[i] <= 1.8:
    #         nonIdealActual.append(labels[i])
    #         nonIdealPred.append(predictions[i])
    #         if predictions[i] <= 1.8:
    #             nonIdealCorrect += 1
    #         else:
    #             nonIdealIncorrect += 1
    #     elif labels[i] < 2.0:
    #         unviableActual.append(labels[i])
    #         unviablePred.append(predictions[i])
    #         if predictions[i] < 2.0:
    #             unviableCorrect += 1
    #         else:
    #             unviableIncorrect += 1

    # # print(f"Good: {goodCorrect} correct, {goodIncorrect} incorrect")
    # # print(f"Non-Ideal: {nonIdealCorrect} correct, {nonIdealIncorrect} incorrect")
    # # print(f"Unviable: {unviableCorrect} correct, {unviableIncorrect} incorrect")
    # # print(f"Good: {goodCorrect/(goodCorrect+goodIncorrect):.2f} correct")
    # # print(f"Non-Ideal: {nonIdealCorrect/(nonIdealCorrect+nonIdealIncorrect):.2f} correct")
    # # print(f"Unviable: {unviableCorrect/(unviableCorrect+unviableIncorrect):.2f} correct")

    # # Confusion matrix assembly (only for good and non-ideal)
    # totalGood = goodCorrect + goodIncorrect
    # totalNonIdeal = nonIdealCorrect + nonIdealIncorrect
    # mtx = np.array([[goodCorrect, goodIncorrect],
    #                 [nonIdealIncorrect, nonIdealCorrect]])
    # mtx = mtx.astype('float') / mtx.sum(axis=1)[:, np.newaxis]  # Normalize by row (i.e. by the number of samples in each class)
    # print(mtx)

    # goodActual = np.array(goodActual)
    # goodPred = np.array(goodPred)
    # nonIdealActual = np.array(nonIdealActual)
    # nonIdealPred = np.array(nonIdealPred)
    # unviableActual = np.array(unviableActual)
    # unviablePred = np.array(unviablePred)

    # # Confusion matrix plot (CoPilot go brr...)
    # plt.figure(figsize=(8, 8))
    # plt.subplot(1, 2, 1)
    # plt.imshow(mtx, interpolation='nearest', cmap=plt.cm.Blues)
    # plt.title('Confusion matrix')
    # plt.colorbar()
    # tick_marks = np.arange(len(["Good", "Non-Ideal"]))
    # plt.xticks(tick_marks, ["Good", "Non-Ideal"], rotation=45)
    # plt.yticks(tick_marks, ["Good", "Non-Ideal"])
    # plt.tight_layout()
    # plt.ylabel('True label')    


    # # Box and whisker plots for each class (CoPilot again)
    # plt.figure(figsize=(10, 6))
    # plt.subplot(1, 3, 1)
    # plt.boxplot(goodActual, positions=[1], widths=0.5)
    # plt.boxplot(goodPred, positions=[2], widths=0.5)
    # plt.title("Good")
    # plt.xticks([1, 2], ["Actual", "Predicted"])
    # plt.ylabel("Bulk Density (g/cm^3)")
    # plt.ylim([0, 2.5])
    # plt.subplot(1, 3, 2)
    # plt.boxplot(nonIdealActual, positions=[1], widths=0.5)
    # plt.boxplot(nonIdealPred, positions=[2], widths=0.5)
    # plt.title("Non-Ideal")
    # plt.xticks([1, 2], ["Actual", "Predicted"])
    # plt.ylabel("Bulk Density (g/cm^3)")
    # plt.ylim([0, 2.5])
    # plt.subplot(1, 3, 3)
    # plt.boxplot(unviableActual, positions=[1], widths=0.5)
    # plt.boxplot(unviablePred, positions=[2], widths=0.5)
    # plt.title("Unviable")
    # plt.xticks([1, 2], ["Actual", "Predicted"])
    # plt.ylabel("Bulk Density (g/cm^3)")
    # plt.ylim([0, 2.5])
    # plt.tight_layout()
    # plt.show()