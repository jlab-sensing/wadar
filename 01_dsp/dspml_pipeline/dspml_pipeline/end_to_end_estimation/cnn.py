"""End-to-end regression using pretrained CNN."""

import logging
logger = logging.getLogger(__name__)
logging.getLogger('absl').setLevel(logging.ERROR)

import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.data import Dataset
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error, mean_absolute_error
import time
import os
from PIL import Image

from ..parameters import RANDOM_SEED, KFOLD_SPLITS, num2label

tf.random.set_seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

class CNNEstimator:
    """
    CNN-based regression model for radar signal analysis using pretrained MobileNetV2.
    
    Attributes:
        kfold_splits (int):         Number of splits for K-fold cross-validation.
        model (keras.Model):        The Keras regression model (MobileNetV2-based).
        base_model (keras.Model):   The pretrained MobileNetV2 base model.
        history (History):          Training history from Keras fit().
        img_size (tuple):           Target image size (height, width) for CNN input.
        batch_size (int):           Batch size for model training and inference.
        epochs (int):               Number of training epochs.
        verbose (int):              Verbosity level for training.
        X_raw (np.ndarray):         Raw complex radar data.
        y (np.ndarray):             Target regression values.
        output_dir (str):           Directory for saving temporary images.
    """
    
    def __init__(self, X: np.ndarray, y: np.ndarray, 
                 output_dir: str = None, img_size: tuple = (160, 160), 
                 batch_size: int = 32, epochs: int = 50, 
                 verbose: int = 0):
        """
        Initialize the CNN-based regression model.

        Args:
            X (np.ndarray):         Complex radar data of shape (samples, range_bins, slow_time).
            y (np.ndarray):         Target regression values.
            output_dir (str):       Directory to save temporary images (default: "/tmp/cnn_radar_images").
            img_size (tuple):       Target image size (height, width) for CNN input (default: (160, 160)).
            batch_size (int):       Batch size for model training and inference (default: 32).
            epochs (int):           Number of training epochs (default: 50).
            verbose (int):          Verbosity level for training (default: 0).
        """
        
        self.kfold_splits = KFOLD_SPLITS
        self.model = None
        self.base_model = None
        self.history = None
        
        self.img_size = img_size
        self.batch_size = batch_size
        self.epochs = epochs
        self.verbose = verbose
        
        # Store raw data for cross-validation
        self.X_raw = X
        self.y = y
        
        # Set up output directory for temporary images
        if output_dir is None:
            output_dir = "/tmp/cnn_radar_images"
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

    def full_monty(self):
        """
        Performs the entire feature regression process.

        Returns:
            model (keras.Model):        Trained Keras model.
            metrics (dict):             Cross-validation metrics.
        """
        
        logger.info(f"Processing complex radar data for CNN...")
        
        # Perform cross-validation
        metrics = self.cross_validate()
        
        # Train final model on all data
        self.model = self.train_final_model()
        
        return self.model, metrics

    def _process_complex_data(self, X):
        
        images = []
        
        for idx, sample in enumerate(X):
            # Convert complex data to amplitude
            amplitude = np.abs(sample)
            
            # Normalize to 0-255 range
            sample_min = amplitude.min()
            sample_max = amplitude.max()
            normalized_sample = 255 * (amplitude - sample_min) / (sample_max - sample_min + 1e-8)
            
            # Convert to PIL Image and resize
            img = Image.fromarray(normalized_sample.astype(np.uint8)).convert('RGB')
            img = img.resize(self.img_size)
            images.append(np.array(img))
            
        return np.stack(images)

    def build_model(self):
        
        # Rescale pixel values from [0, 255] to [-1, 1]
        rescale = keras.layers.Rescaling(1./127.5, offset=-1)
        
        # Create base model from pretrained MobileNetV2
        IMG_SHAPE = self.img_size + (3,)
        self.base_model = keras.applications.MobileNetV2(
            input_shape=IMG_SHAPE,
            include_top=False,
            weights='imagenet'
        )
        
        # Freeze base model initially
        self.base_model.trainable = False
        
        # Build full model
        inputs = keras.Input(shape=IMG_SHAPE)
        x = rescale(inputs)
        x = self.base_model(x, training=False)
        x = keras.layers.GlobalAveragePooling2D()(x)
        x = keras.layers.Dropout(0.2)(x)
        outputs = keras.layers.Dense(1)(x)
        
        self.model = keras.Model(inputs, outputs)
        
        self.model.compile(
            optimizer='adam',
            loss='mse',
            metrics=['mse', 'mae']
        )
        
        return self.model

    def cross_validate(self):
        
        # Process complex data to images
        X_images = self._process_complex_data(self.X_raw)
        
        kfold = KFold(n_splits=self.kfold_splits, shuffle=True, random_state=RANDOM_SEED)
        
        cv_mse_scores = []
        cv_mae_scores = []
        cv_rmse_scores = []
        cv_accuracy_scores = []
        fold_times = []
        fold_training_times = []
        fold_inference_times = []
        
        logger.info(f"Starting {self.kfold_splits}-fold cross-validation...")
        
        for fold, (train_idx, val_idx) in enumerate(kfold.split(X_images)):
            fold_start_time = time.time()
            
            # Split data
            X_train_fold = X_images[train_idx]
            X_val_fold = X_images[val_idx]
            y_train_fold = self.y[train_idx]
            y_val_fold = self.y[val_idx]
            
            # Create datasets
            train_ds = Dataset.from_tensor_slices((X_train_fold, y_train_fold)).batch(self.batch_size)
            val_ds = Dataset.from_tensor_slices((X_val_fold, y_val_fold)).batch(self.batch_size)
            
            AUTOTUNE = tf.data.AUTOTUNE
            train_ds = train_ds.prefetch(buffer_size=AUTOTUNE)
            val_ds = val_ds.prefetch(buffer_size=AUTOTUNE)
            
            # Build and train model
            tf.keras.backend.clear_session()
            if fold == 0:
                model = self.build_model()
                initial_weights = model.get_weights()
            model.set_weights(initial_weights)
            
            # Train with early stopping
            callbacks = [
                keras.callbacks.EarlyStopping(
                    patience=20, restore_best_weights=True, monitor='val_loss'
                ),
                keras.callbacks.ReduceLROnPlateau(
                    monitor='val_loss', factor=0.5, patience=10, min_lr=1e-6
                )
            ]
            
            # Track training time
            training_start_time = time.time()
            
            # Initial training with frozen base model
            initial_epochs = self.epochs // 2
            model.fit(
                train_ds,
                validation_data=val_ds,
                epochs=initial_epochs,
                verbose=self.verbose,
                callbacks=callbacks
            )
            
            # Fine-tuning
            fine_tune_at = 100
            self.base_model.trainable = True
            for layer in self.base_model.layers[:fine_tune_at]:
                layer.trainable = False
                
            model.compile(
                optimizer=keras.optimizers.Adam(1e-5),
                loss='mse',
                metrics=['mse', 'mae']
            )
            
            fine_tune_epochs = self.epochs - initial_epochs
            model.fit(
                train_ds,
                validation_data=val_ds,
                epochs=fine_tune_epochs,
                verbose=self.verbose,
                callbacks=callbacks
            )
            
            training_time = time.time() - training_start_time
            fold_training_times.append(training_time)
            
            # Track inference time
            inference_start_time = time.time()
            y_pred_fold = model.predict(X_val_fold, verbose=0).flatten()
            inference_time = time.time() - inference_start_time
            fold_inference_times.append(inference_time)
            
            # Calculate metrics
            mse = mean_squared_error(y_val_fold, y_pred_fold)
            mae = mean_absolute_error(y_val_fold, y_pred_fold)
            rmse = np.sqrt(mse)
            
            # Calculate accuracy using num2label
            y_labels = [num2label(label) for label in y_val_fold]
            y_pred_labels = [num2label(pred) for pred in y_pred_fold]
            accuracy = np.mean([pred_label == true_label for pred_label, true_label in zip(y_pred_labels, y_labels)])
            
            cv_mse_scores.append(mse)
            cv_mae_scores.append(mae)
            cv_rmse_scores.append(rmse)
            cv_accuracy_scores.append(accuracy)
            
            fold_time = time.time() - fold_start_time
            fold_times.append(fold_time)
            
            logger.info(f"Fold {fold+1}/{KFOLD_SPLITS} - MAE: {mae:.2f}, RMSE: {rmse:.2f}, "
                        f"Accuracy: {100*accuracy:.2f}%, Training time: {1000*training_time:.2f}ms, "
                        f"Inference time: {1000*inference_time:.2f}ms")
        
        # Calculate average metrics
        metrics = {
            'mae': np.mean(cv_mae_scores),
            'mse': np.mean(cv_mse_scores),
            'rmse': np.mean(cv_rmse_scores),
            'accuracy': np.mean(cv_accuracy_scores),
            'training_time': np.mean(fold_training_times),
            'inference_time': np.mean(fold_inference_times),
            'total_time': np.sum(fold_times)
        }
        
        logger.info(f"Average metrics - MAE: {metrics['mae']:.2f}, RMSE: {metrics['rmse']:.2f}, "
                   f"Accuracy: {100*metrics['accuracy']:.2f}%, Training time: {1000*metrics['training_time']:.2f}ms, "
                   f"Inference time: {1000*metrics['inference_time']:.2f}ms")
        
        return metrics

    def train_final_model(self):
        
        # Process complex data to images
        X_images = self._process_complex_data(self.X_raw)
        
        # Create dataset
        train_ds = Dataset.from_tensor_slices((X_images, self.y)).batch(self.batch_size)
        AUTOTUNE = tf.data.AUTOTUNE
        train_ds = train_ds.prefetch(buffer_size=AUTOTUNE)
        
        # Build and train final model
        self.model = self.build_model()
        
        callbacks = [
            keras.callbacks.EarlyStopping(
                patience=20, restore_best_weights=True, monitor='loss'
            ),
            keras.callbacks.ReduceLROnPlateau(
                monitor='loss', factor=0.5, patience=10, min_lr=1e-6
            )
        ]
        
        # Initial training with frozen base model
        initial_epochs = self.epochs // 2
        self.history = self.model.fit(
            train_ds,
            epochs=initial_epochs,
            verbose=self.verbose,
            callbacks=callbacks
        )
        
        # Fine-tuning
        fine_tune_at = 100
        self.base_model.trainable = True
        for layer in self.base_model.layers[:fine_tune_at]:
            layer.trainable = False
            
        self.model.compile(
            optimizer=keras.optimizers.Adam(1e-5),
            loss='mse',
            metrics=['mse', 'mae']
        )
        
        fine_tune_epochs = self.epochs - initial_epochs
        self.history = self.model.fit(
            train_ds,
            epochs=fine_tune_epochs,
            verbose=self.verbose,
            callbacks=callbacks
        )
        
        return self.model

    def estimate(self, X : np.ndarray):
        """
        Estimates target values for the given input features using the trained model.

        Args:
            X (np.ndarray): Input features for prediction.

        Returns:
            np.ndarray:     Predicted target values.
        """
        
        if self.model is None:
            raise ValueError("Model not trained. Call train_final_model first.")
        
        # Process complex data to images
        X_images = self._process_complex_data(X)
        
        return self.model.predict(X_images, verbose=0)