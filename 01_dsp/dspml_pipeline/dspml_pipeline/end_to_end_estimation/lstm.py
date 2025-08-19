import logging
logger = logging.getLogger(__name__)
logging.getLogger('absl').setLevel(logging.ERROR)

import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error, mean_absolute_error
import time

from ..parameters import RANDOM_SEED, KFOLD_SPLITS, num2label

# Set seeds for reproducibility
tf.random.set_seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)


class LSTMEstimator:
    """
    LSTM-based regression model for radar signal analysis.

    Attributes:
        kfold_splits (int):     Number of splits for K-fold cross-validation.
        model (keras.Model):    The Keras regression model (LSTM-based).
        history (History):      Training history from Keras fit().
        scaler (MinMaxScaler):  Scaler used for feature normalization.
        epochs (int):           Number of training epochs.
        batch_size (int):       Batch size for model training and inference.
        verbose (int):          Verbosity level for training.
        X_raw (np.ndarray):     Raw complex radar data.
        y (np.ndarray):         Target regression values.
    """
    
    def __init__(self, X : np.ndarray, y : np.ndarray, 
                 epochs : int = 50, batch_size : int =16, 
                 verbose : bool =0):
        """
        Initialize LSTM regression model.
        
        Args:
            X (np.ndarray):         Complex radar data of shape (samples, range_bins, slow_time).
            y (np.ndarray):         Target regression values.
            epochs (int):           Number of training epochs. Default is 50.
            batch_size (int):       Batch size for training and inference. Default is 16.
            verbose (bool):         Verbosity level for training. Default is 0.
        """

        self.kfold_splits = KFOLD_SPLITS
        self.model = None
        self.history = None
        self.scaler = MinMaxScaler()

        self.epochs = epochs
        self.batch_size = batch_size
        self.verbose = verbose

        # Store raw data for cross-validation
        self.X_raw = X
        self.y = y

    def full_monty(self):
        """
        Performs the entire feature regression process.

        Returns:
            model (keras.Model):        Trained Keras model.
            metrics (dict):             Cross-validation metrics.
        """

        logger.info(f"Processing complex radar data for LSTM...")

        # Perform cross-validation
        metrics = self.cross_validate()
        
        # Process complex data to LSTM-friendly format
        self.model = self.train_final_model()
                                            
        return self.model, metrics
        
    def _process_complex_data(self, X):

        # Extract amplitude and phase
        X_amplitude = np.abs(X) 
        X_phase = np.angle(X)   
        
        # Transpose to treat slow_time as timesteps and range_bins as features so that it tracks
        # the changes in signal over slow time
        X_amp_seq = X_amplitude.transpose(0, 2, 1)  # (samples, slow_time, range_bins)
        X_phase_seq = X_phase.transpose(0, 2, 1)   
        
        # Stack amplitude and phase as features
        X_features = np.concatenate([X_amp_seq, X_phase_seq], axis=2) # (samples, slow_time, range_bins * 2)
        
        return X_features
    
    def _normalize_data(self, X_train, X_val=None, fit_scaler=True):

        # Flatten for normalization, then reshape back
        n_samples, n_timesteps, n_features = X_train.shape
        X_train_flat = X_train.reshape(n_samples, -1)
        
        if fit_scaler:
            X_train_norm = self.scaler.fit_transform(X_train_flat)
        else:
            X_train_norm = self.scaler.transform(X_train_flat)
        
        X_train_norm = X_train_norm.reshape(n_samples, n_timesteps, n_features)
        
        if X_val is not None:
            n_val_samples, n_val_timesteps, n_val_features = X_val.shape
            X_val_flat = X_val.reshape(n_val_samples, -1)
            X_val_norm = self.scaler.transform(X_val_flat)
            X_val_norm = X_val_norm.reshape(n_val_samples, n_val_timesteps, n_val_features)
            return X_train_norm, X_val_norm
        
        return X_train_norm
    
    def build_model(self, input_shape):
        
        timesteps, features = input_shape
        self.model = keras.Sequential([
            layers.Input(shape=(timesteps, features)),
            layers.LSTM(32),
            layers.Dense(16, activation='relu'),
            layers.Dense(1)
        ])
        
        self.model.compile(
            optimizer='adam',
            loss='mse',
            metrics=['mse', 'mae']
        )
        
        return self.model
    
    def cross_validate(self):

        # Use raw processed data (before normalization) for cross-validation
        X_processed = self._process_complex_data(self.X_raw)
        
        kfold = KFold(n_splits=self.kfold_splits, shuffle=True, random_state=RANDOM_SEED)
        
        cv_mse_scores = []
        cv_mae_scores = []
        cv_rmse_scores = []
        cv_accuracy_scores = []
        fold_times = []
        fold_training_times = []
        fold_inference_times = []
        
        logger.info(f"Starting {self.kfold_splits}-fold cross-validation...")
        
        for fold, (train_idx, val_idx) in enumerate(kfold.split(X_processed)):
            fold_start_time = time.time()
            
            # Split data
            X_train_fold = X_processed[train_idx]
            X_val_fold = X_processed[val_idx]
            y_train_fold = self.y[train_idx]
            y_val_fold = self.y[val_idx]
            
            # Create a new scaler for this fold to prevent data leakage
            fold_scaler = MinMaxScaler()
            
            # Normalize data for this fold
            n_train_samples, n_timesteps, n_features = X_train_fold.shape
            X_train_flat = X_train_fold.reshape(n_train_samples, -1)
            X_train_norm = fold_scaler.fit_transform(X_train_flat)
            X_train_norm = X_train_norm.reshape(n_train_samples, n_timesteps, n_features)
            X_train_norm = tf.convert_to_tensor(X_train_norm, dtype=tf.float32)
            
            n_val_samples = X_val_fold.shape[0]
            X_val_flat = X_val_fold.reshape(n_val_samples, -1)
            X_val_norm = fold_scaler.transform(X_val_flat)
            X_val_norm = X_val_norm.reshape(n_val_samples, n_timesteps, n_features)
            X_val_norm = tf.convert_to_tensor(X_val_norm, dtype=tf.float32)

            # Build and train model just once
            if fold == 0:
                model = self.build_model((X_train_norm.shape[1], X_train_norm.shape[2]))
                self.initial_weights = model.get_weights()
            model.set_weights(self.initial_weights)
            
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
            model.fit(
                X_train_norm, y_train_fold,
                validation_data=(X_val_norm, y_val_fold),
                epochs=self.epochs,
                batch_size=self.batch_size,
                verbose=self.verbose,
                callbacks=callbacks
            )
            training_time = time.time() - training_start_time
            fold_training_times.append(training_time)
            
            # Track inference time
            inference_start_time = time.time()
            y_pred_fold = model.predict(X_val_norm, batch_size=self.batch_size, verbose=self.verbose).flatten()
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
        
        # Process complex data to amplitude/phase features
        X_features = self._process_complex_data(self.X_raw)
        # Normalize using the scaler (fit on all data)
        X_norm = self._normalize_data(X_features, fit_scaler=True)

        # Build and train final model
        self.model.set_weights(self.initial_weights)

        callbacks = [
            keras.callbacks.EarlyStopping(
            patience=20, restore_best_weights=True, monitor='loss'
            ),
            keras.callbacks.ReduceLROnPlateau(
            monitor='loss', factor=0.5, patience=10, min_lr=1e-6
            )
        ]
        
        self.history = self.model.fit(
            X_norm, self.y,
            epochs=self.epochs,
            batch_size=self.batch_size,
            verbose=self.verbose,
            callbacks=callbacks
        )
        
        return self.model
    
    def estimate(self, X):
        """
        Estimates target values for the given input features using the trained model.

        Args:
            X (np.ndarray): Input features for prediction.

        Returns:
            np.ndarray:     Predicted target values.
        """

        if self.model is None:
            raise ValueError("Model not trained. Call train_final_model first.")
        
        # Process and normalize data using the fitted scaler
        X_features = self._process_complex_data(X)
        
        # Use the same scaler fitted during training
        n_samples, n_timesteps, n_features = X_features.shape
        X_flat = X_features.reshape(n_samples, -1)
        X_norm = self.scaler.transform(X_flat)
        X_norm = X_norm.reshape(n_samples, n_timesteps, n_features)
        
        return self.model.predict(X_norm, batch_size=self.batch_size, verbose=self.verbose)