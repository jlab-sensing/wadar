import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
import time
from _06_hermes.parameters import RANDOM_SEED, KFOLD_SPLITS, num2label

# Set seeds for reproducibility
tf.random.set_seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)


class LSTMRegressor:
    """
    LSTM-based regression model for radar signal analysis.
    """
    
    def __init__(self, kfold_splits=KFOLD_SPLITS):
        """
        Initialize LSTM regressor.
        
        Parameters:
            kfold_splits: Number of folds for cross-validation
        """

        self.kfold_splits = kfold_splits
        self.model = None
        self.history = None
        self.scaler = MinMaxScaler()
        
    def _process_complex_data(self, X_complex):
        """
        Convert complex radar data to amplitude/phase features for LSTM.
        
        Components:
            X_complex: Complex radar data (samples, fast_time, slow_time)
            
        Returns:
            Processed features (samples, timesteps, features)
        """

        # Extract amplitude and phase
        X_amplitude = np.abs(X_complex) 
        X_phase = np.angle(X_complex)   
        
        # Transpose to treat slow_time as timesteps and range_bins as features so that it tracks
        # the changes in signal over slow time
        X_amp_seq = X_amplitude.transpose(0, 2, 1)  # (samples, slow_time, range_bins)
        X_phase_seq = X_phase.transpose(0, 2, 1)   
        
        # Stack amplitude and phase as features
        X_features = np.concatenate([X_amp_seq, X_phase_seq], axis=2) # (samples, slow_time, range_bins * 2)
        
        return X_features
    
    def _normalize_data(self, X_train, X_val=None):
        """
        Normalize training and validation data.
        
        Components:
            X_train: Training data (samples, timesteps, features)
            X_val: Validation data (optional)
            
        Returns:
            Normalized data
        """

        # Flatten for normalization, then reshape back
        n_samples, n_timesteps, n_features = X_train.shape
        X_train_flat = X_train.reshape(n_samples, -1)
        
        X_train_norm = self.scaler.fit_transform(X_train_flat)
        X_train_norm = X_train_norm.reshape(n_samples, n_timesteps, n_features)
        
        if X_val is not None:
            n_val_samples = X_val.shape[0]
            X_val_flat = X_val.reshape(n_val_samples, -1)
            X_val_norm = self.scaler.transform(X_val_flat)
            X_val_norm = X_val_norm.reshape(n_val_samples, n_timesteps, n_features)
            return X_train_norm, X_val_norm
        
        return X_train_norm
    
    def build_model(self, input_shape, lstm_units_1=64, lstm_units_2=32, 
                   dense_units_1=32, dense_units_2=16, dropout_rate=0.2):
        """
        Build LSTM architecture for radar data regression.
        
        Components:
            input_shape: Input shape (timesteps, features)
            lstm_units_1: Units in first LSTM layer
            lstm_units_2: Units in second LSTM layer
            dense_units_1: Units in first dense layer
            dense_units_2: Units in second dense layer
            dropout_rate: Dropout rate
            
        Returns:
            Compiled Keras model
        """
        
        timesteps, features = input_shape
        model = keras.Sequential([
            layers.LSTM(32, input_shape=(timesteps, features)),
            layers.Dense(16, activation='relu'),
            layers.Dense(1)
        ])
        
        model.compile(
            optimizer='adam',
            loss='mae',  # Mean Absolute Error for regression
            metrics=['mse', 'mae']
        )
        
        return model
    
    def _cross_validate(self, X_features, y, epochs=50, batch_size=16, verbose=0):
        """
        Perform cross-validation to evaluate model performance.
        
        Components:
            X_features: Processed feature array (samples, timesteps, features)
            y: Target labels
            epochs: Number of training epochs
            batch_size: Training batch size
            verbose: Verbosity level
            
        Returns:
            Dictionary of cross-validation metrics
        """

        kfold = KFold(n_splits=self.kfold_splits, shuffle=True, random_state=RANDOM_SEED)
        
        cv_mse_scores = []
        cv_r2_scores = []
        cv_mae_scores = []
        cv_rmse_scores = []
        cv_accuracy_scores = []
        fold_times = []
        fold_training_times = []
        fold_inference_times = []
        
        print(f"[INFO] Starting {self.kfold_splits}-fold cross-validation...")
        
        for fold, (train_idx, val_idx) in enumerate(kfold.split(X_features)):
            fold_start_time = time.time()
            
            # Split data
            X_train_fold = X_features[train_idx]
            X_val_fold = X_features[val_idx]
            y_train_fold = y[train_idx]
            y_val_fold = y[val_idx]
            
            # Normalize data for this fold
            X_train_norm, X_val_norm = self._normalize_data(X_train_fold, X_val_fold)
            
            # Build and train model
            model = self.build_model((X_train_norm.shape[1], X_train_norm.shape[2]))
            
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
                epochs=epochs,
                batch_size=batch_size,
                verbose=verbose,
                callbacks=callbacks
            )
            training_time = time.time() - training_start_time
            fold_training_times.append(training_time)
            
            # Track inference time
            inference_start_time = time.time()
            y_pred_fold = model.predict(X_val_norm, verbose=0).flatten()
            inference_time = time.time() - inference_start_time
            fold_inference_times.append(inference_time)
            
            # Calculate metrics
            mse = mean_squared_error(y_val_fold, y_pred_fold)
            r2 = r2_score(y_val_fold, y_pred_fold)
            mae = mean_absolute_error(y_val_fold, y_pred_fold)
            rmse = np.sqrt(mse)
            
            # Calculate accuracy using num2label
            y_labels = [num2label(label) for label in y_val_fold]
            y_pred_labels = [num2label(pred) for pred in y_pred_fold]
            accuracy = np.mean([pred_label == true_label for pred_label, true_label in zip(y_pred_labels, y_labels)])
            
            cv_mse_scores.append(mse)
            cv_r2_scores.append(r2)
            cv_mae_scores.append(mae)
            cv_rmse_scores.append(rmse)
            cv_accuracy_scores.append(accuracy)
            
            fold_time = time.time() - fold_start_time
            fold_times.append(fold_time)
            
            print(f"[INFO] Fold {fold + 1}/{self.kfold_splits} - "
                  f"MAE: {mae:.6f}, R²: {r2:.6f}, RMSE: {rmse:.6f}, Acc: {accuracy:.6f}, Time: {fold_time:.2f}s")
        
        # Calculate average metrics
        metrics = {
            'mae': np.mean(cv_mae_scores),
            'mse': np.mean(cv_mse_scores),
            'r2': np.mean(cv_r2_scores),
            'rmse': np.mean(cv_rmse_scores),
            'accuracy': np.mean(cv_accuracy_scores),
            'training_time': np.mean(fold_training_times),
            'inference_time': np.mean(fold_inference_times),
            'total_time': np.sum(fold_times)
        }
        
        return metrics
    
    def full_monty_eval(self, X_complex, y, epochs=50, batch_size=16, verbose=0):
        """
        Complete LSTM evaluation pipeline: data processing + cross-validation.
        
        Components:
            X_complex: Complex radar data (samples, range_bins, slow_time)
            y: Target labels
            epochs: Number of training epochs
            batch_size: Training batch size
            verbose: Verbosity level
            
        Returns:
            Dictionary of cross-validation metrics
        """

        print(f"[INFO] Processing complex radar data for LSTM...")
        print(f"[INFO] Input shape: {X_complex.shape}")
        
        # Process complex data to LSTM-friendly format
        X_features = self._process_complex_data(X_complex)
        print(f"[INFO] Processed shape: {X_features.shape}")
        print(f"[INFO] Timesteps: {X_features.shape[1]}, Features: {X_features.shape[2]}")
        
        # Perform cross-validation
        metrics = self._cross_validate(X_features, y, epochs, batch_size, verbose)
        
        return metrics
    
    def train_final_model(self, X_complex, y, epochs=50, batch_size=16, verbose=1):
        """
        Train final model on all data.
        
        Components:
            X_complex: Complex radar data
            y: Target labels
            epochs: Number of training epochs
            batch_size: Training batch size
            verbose: Verbosity level
            
        Returns:
            Trained model
        """

        # Process data
        X_features = self._process_complex_data(X_complex)
        X_norm = self._normalize_data(X_features)
        
        # Build and train final model
        self.model = self.build_model((X_norm.shape[1], X_norm.shape[2]))
        
        callbacks = [
            keras.callbacks.EarlyStopping(
                patience=20, restore_best_weights=True, monitor='loss'
            ),
            keras.callbacks.ReduceLROnPlateau(
                monitor='loss', factor=0.5, patience=10, min_lr=1e-6
            )
        ]
        
        self.history = self.model.fit(
            X_norm, y,
            epochs=epochs,
            batch_size=batch_size,
            verbose=verbose,
            callbacks=callbacks
        )
        
        return self.model
    
    def predict(self, X_complex):
        """
        Make predictions on new data.
        
        Components:
            X_complex: Complex radar data
            
        Returns:
            Predictions
        """

        if self.model is None:
            raise ValueError("Model not trained. Call train_final_model first.")
        
        # Process and normalize data
        X_features = self._process_complex_data(X_complex)
        
        # Use the same scaler fitted during training
        n_samples, n_timesteps, n_features = X_features.shape
        X_flat = X_features.reshape(n_samples, -1)
        X_norm = self.scaler.transform(X_flat)
        X_norm = X_norm.reshape(n_samples, n_timesteps, n_features)
        
        return self.model.predict(X_norm, verbose=0)
