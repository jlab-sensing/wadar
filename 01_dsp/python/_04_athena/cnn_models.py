from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split, KFold
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, Flatten, Dense, Dropout, MaxPooling1D
from _05_apollo import viz_tools
from _06_hermes.parameters import num2label, RANDOM_SEED, KFOLD_SPLITS
import tensorflow as tf
import time
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

tf.random.set_seed(RANDOM_SEED)

class CNN1D:
    """
    1D Convolutional Neural Network for regression on radar data.
    
    Processes complex radar signals by extracting amplitude and phase components,
    then applies 1D CNN for bulk density prediction.
    """
    
    def __init__(self, X, y, n_splits=KFOLD_SPLITS):
        """
        Initialize CNN1D model.
        
        Args:
            X: Complex radar data of shape (samples, range_bins, slow_time)
            y: Target labels (bulk density values)
            n_splits: Number of splits for cross-validation
        """
        self.X_raw = X
        self.y = y
        self.n_splits = n_splits
        self.model = None
        
        self.X = X
        
    def scale_each_sample(self, X):
        """Scale each sample independently to [0,1] range (same as demo normalization approach)."""
        # Flatten each sample for normalization, then reshape back
        n_samples, n_bins, n_features = X.shape
        
        X_flat = X.reshape(n_samples, -1)
        X_scaled_flat = np.zeros_like(X_flat)
        
        for i in range(n_samples):
            sample_flat = X_flat[i]
            if sample_flat.max() != sample_flat.min():  # Avoid division by zero
                X_scaled_flat[i] = (sample_flat - sample_flat.min()) / (sample_flat.max() - sample_flat.min())
            else:
                X_scaled_flat[i] = sample_flat
        
        # Reshape back to original shape
        X_scaled = X_scaled_flat.reshape(n_samples, n_bins, n_features)
        return X_scaled
    
    def full_monty(self, epochs=30, batch_size=16):
        """
        Complete training pipeline with cross-validation and final model training.
        
        Args:
            epochs: Number of training epochs
            batch_size: Training batch size
            
        Returns:
            Trained model and cross-validation metrics
        """
        print(f"[INFO] Starting 1D CNN training with {epochs} epochs, batch size {batch_size}")
        
        # Scale the data
        X_scaled = self.scale_each_sample(self.X)
        
        # Cross-validate to get performance estimates
        metrics = self.cross_validate(X_scaled, epochs=epochs, batch_size=batch_size)
        
        # Train final model on all data
        model = self.train(X_scaled, epochs=epochs, batch_size=batch_size)
        self.model = model 

        return model, metrics

    def build_model(self, input_shape, filters=32, dense_units=32):
        """
        Build 1D CNN architecture for radar data regression (same as demo).
        
        Args:
            input_shape: Shape of input data (range_bins, channels)
            filters: Number of convolutional filters
            dense_units: Number of units in dense layers
        """
        model = Sequential([
            # First conv block
            Conv1D(32, kernel_size=5, activation='relu', input_shape=input_shape),
            Conv1D(32, kernel_size=3, activation='relu'),
            MaxPooling1D(pool_size=2),
            Dropout(0.2),
            
            # Second conv block  
            Conv1D(64, kernel_size=3, activation='relu'),
            MaxPooling1D(pool_size=2),
            Dropout(0.2),
            
            # Dense layers
            Flatten(),
            Dense(32, activation='relu'),
            Dropout(0.3),
            Dense(16, activation='relu'),
            Dense(1, activation='linear')  # Single output for regression
        ])
        
        model.compile(
            optimizer='adam', 
            loss='mae',
            metrics=['mse', 'mae']
        )
        return model

    def train(self, X_scaled, epochs=30, batch_size=16):
        """Train the final model on all available data."""
        input_shape = (X_scaled.shape[1], X_scaled.shape[2])
        self.model = self.build_model(input_shape=input_shape)
        
        # Add early stopping
        callbacks = [
            tf.keras.callbacks.EarlyStopping(
                patience=10, 
                restore_best_weights=True,
                monitor='loss'
            ),
            tf.keras.callbacks.ReduceLROnPlateau(
                patience=5, 
                factor=0.5, 
                min_lr=1e-6
            )
        ]
        
        self.model.fit(
            X_scaled, self.y, 
            epochs=epochs, 
            batch_size=batch_size, 
            verbose=0,
            callbacks=callbacks
        )
        return self.model
    
    def cross_validate(self, X_scaled, epochs=30, batch_size=16):
        """Perform k-fold cross-validation to estimate model performance."""
        kf = KFold(n_splits=self.n_splits, shuffle=True, random_state=RANDOM_SEED)

        mae_scores = []
        inference_times = []
        accuracy_scores = []
        rmse_scores = []
        training_times = []
        r2_scores = []

        print(f"[INFO] Performing {self.n_splits}-fold cross-validation...")

        for fold, (train_index, val_index) in enumerate(kf.split(X_scaled)):
            print(f"[INFO] Training fold {fold + 1}/{self.n_splits}")
            
            X_train, X_val = X_scaled[train_index], X_scaled[val_index]
            y_train, y_val = self.y[train_index], self.y[val_index]

            # Build and train model for this fold
            input_shape = (X_train.shape[1], X_train.shape[2])
            model = self.build_model(input_shape=input_shape)
            
            callbacks = [
                tf.keras.callbacks.EarlyStopping(
                    patience=25, 
                    restore_best_weights=True,
                    monitor='val_loss',
                    verbose=0
                ),
                tf.keras.callbacks.ReduceLROnPlateau(
                    patience=10, 
                    factor=0.5, 
                    min_lr=1e-6,
                    verbose=0
                )
            ]
            
            time_start = time.time()
            model.fit(
                X_train, y_train, 
                epochs=epochs, 
                batch_size=batch_size, 
                verbose=0,
                validation_data=(X_val, y_val),
                callbacks=callbacks
            )
            training_times.append(time.time() - time_start)

            # Measure inference time
            time_start = time.time()
            y_pred = model.predict(X_val, verbose=0).flatten()
            inference_times.append(time.time() - time_start)

            # Calculate metrics
            mae_scores.append(mean_absolute_error(y_val, y_pred))
            rmse_scores.append(np.sqrt(mean_squared_error(y_val, y_pred)))
            r2_scores.append(r2_score(y_val, y_pred))

            # Calculate accuracy (categorical)
            y_labels = [num2label(label) for label in y_val]
            pred_labels = [num2label(pred) for pred in y_pred]
            accuracy_scores.append(np.mean(np.array(y_labels) == np.array(pred_labels)))

        metrics = {
            'mae': np.mean(mae_scores),
            'inference_time': np.mean(inference_times),
            'accuracy': np.mean(accuracy_scores),
            'rmse': np.mean(rmse_scores),
            'training_time': np.mean(training_times),
            'r2': np.mean(r2_scores)
        }

        print(f"[INFO] CV Results - MAE: {metrics['mae']:.4f}, R²: {metrics['r2']:.4f}, Accuracy: {metrics['accuracy']:.4f}")
        return metrics

    def predict(self, X):
        """Make predictions on new data."""
        X_processed = X
        X_scaled = self.scale_each_sample(X_processed)
        
        if self.model is None:
            raise ValueError("Model not trained. Call full_monty() first.")
            
        y_pred = self.model.predict(X_scaled, verbose=0).flatten()
        return y_pred

    def save_model(self, model_dir, model_name="cnn1d_regressor.keras"):
        """Save the trained model."""
        if self.model is None:
            raise ValueError("No model to save. Train the model first.")
        
        import os
        os.makedirs(model_dir, exist_ok=True)
        model_path = os.path.join(model_dir, model_name)
        self.model.save(model_path)
        print(f"[INFO] Model saved to {model_path}")

    def load_model(self, model_dir, model_name="cnn1d_regressor.keras"):
        """Load a pre-trained model."""
        import os
        model_path = os.path.join(model_dir, model_name)
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        self.model = tf.keras.models.load_model(model_path)
        print(f"[INFO] Model loaded from {model_path}")
        return self.model