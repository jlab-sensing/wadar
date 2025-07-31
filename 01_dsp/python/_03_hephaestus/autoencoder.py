import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.datasets import load_iris
import keras
from keras.models import Sequential
from keras.layers import Input, Dense
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

class AutoencoderFeatureSelector: 
    """
    AutoencoderFeatureSelector is a class to perform feature selection using an autoencoder.
    It can be used to reduce the dimensionality of the data and extract features that are useful
    for classification tasks. Much of this was sourced from
    https://hex.tech/blog/autoencoders-for-feature-selection/

    Parameters:
        X (np.ndarray):         Input data of shape (samples, features).
        encoding_dim (int):     Dimension of the encoded representation.
        signal_type (str):      Type of signal to process ('magnitude', 'phase', or None for both).
        model_name (str):       Name of the model to save/load.
        model_dir (str):        Directory to save/load the model.
        load_model (bool):      If True, load the model instead of training it.
    """

    def __init__(self, X, encoding_dim, signal_type=None, model_name=None, model_dir=None, load_model=None):
        """
        Initialize the AutoencoderFeatureSelector with the input data and parameters.
        If load_model is True, it will load the model from the specified directory.
        If load_model is False, it will train a new model with the provided data.

        Args:
            X (np.ndarray):         Input data of shape (samples, features).
            encoding_dim (int):     Dimension of the encoded representation.
            signal_type (str):      Type of signal to process ('magnitude', 'phase', or None for both).
            model_name (str):       Name of the model to save/load.
            model_dir (str):        Directory to save/load the model.
            load_model (bool):      If True, load the model instead of training it.
        """

        self.X = self.pre_process(X, signal_type=signal_type)
        self.encoding_dim = encoding_dim
        self.input_dim = self.X.shape[1]
        self.model_name = model_name
        self.model_dir = model_dir

        # The model itself, simple feedforward autoencoder
        input_layer = keras.layers.Input(shape=(self.input_dim,))
        encoder_layer = keras.layers.Dense(self.encoding_dim, activation="relu")(input_layer)
        decoder_layer = keras.layers.Dense(self.input_dim, activation="sigmoid")(encoder_layer)
        self.autoencoder = keras.Model(inputs=input_layer, outputs=decoder_layer)
        self.encoder = keras.Model(inputs=input_layer, outputs=encoder_layer)
        self.autoencoder.compile(optimizer='adam', loss='mae')

    
    def pre_process(self, X, signal_type):
        """
        Pre-process the input data by flattening and optionally extracting magnitude or phase information.
        Apply normalization for better training stability.

        Args:
            X (np.ndarray):         Input data of shape (samples, features) or (samples, range_bins, time_bins).
            signal_type (str):      Type of signal to process ('magnitude', 'phase', or None for both).

        Returns:
            np.ndarray:            Flattened, processed, and normalized input data.
        """
        # Handle different input shapes
        if len(X.shape) == 3:
            N, R, T = X.shape
            X_flat = X.reshape(N, R * T)
        else:
            X_flat = X.copy()
            
        if signal_type == 'magnitude':
            X_flat = np.abs(X_flat)
        elif signal_type == 'phase':
            X_flat = np.angle(X_flat)
        else:       # just stack magnitude and phase
            if len(X.shape) == 3:
                X_mag = np.abs(X).reshape(N, R * T)
                X_phase = np.angle(X).reshape(N, R * T)
            else:
                X_mag = np.abs(X_flat)
                X_phase = np.angle(X_flat)
            X_flat = np.concatenate([X_mag, X_phase], axis=1)

        # Apply normalization for better training stability
        if not hasattr(self, 'scaler'):
            self.scaler = StandardScaler()
            X_flat = self.scaler.fit_transform(X_flat)
        else:
            X_flat = self.scaler.transform(X_flat)

        return X_flat

    def fit(self, epochs=50, batch_size=32, test_size=0.2, patience=10, verbose=0):
        """
        Fit the autoencoder model to the input data with improved training options.
        
        Args:
            epochs (int):          Number of epochs to train the model.
            batch_size (int):      Batch size for training.
            test_size (float):     Proportion of the dataset to include in the test split.
            patience (int):        Early stopping patience.
            verbose (int):         Training verbosity (0=silent, 1=progress bar, 2=one line per epoch).
        
        Returns:
            np.ndarray:            Transformed input data after training the autoencoder.
        """

        if not self.model_name:
            
            X_train, X_test = train_test_split(self.X, test_size=test_size, random_state=42)

            # Add callbacks for better training
            callbacks = [
                keras.callbacks.EarlyStopping(
                    monitor='val_loss', 
                    patience=patience, 
                    restore_best_weights=True,
                    verbose=1 if verbose > 0 else 0
                ),
                keras.callbacks.ReduceLROnPlateau(
                    monitor='val_loss',
                    factor=0.5,
                    patience=patience//2,
                    min_lr=1e-6,
                    verbose=1 if verbose > 0 else 0
                )
            ]

            history = self.autoencoder.fit(
                X_train, X_train,
                epochs=epochs,
                batch_size=batch_size,
                shuffle=True,
                validation_data=(X_test, X_test),
                callbacks=callbacks,
                verbose=verbose
            )
            
            # Store training history for analysis
            self.training_history = history.history

            if verbose > 0:
                final_loss = self.autoencoder.evaluate(X_test, X_test, verbose=0)
                print(f"Final reconstruction loss: {final_loss[0]:.6f}")
        
        else:
            # do not train, just load the model
            self.load_model(self.model_dir, self.model_name)
            

        return self.transform(self.X)

    def transform(self, X):
        """
        Transform the input data using the trained autoencoder model.

        Args:
            X (np.ndarray):         Input data of shape (samples, features).

        Returns:
            np.ndarray:             Transformed input data after passing through the encoder.
        """

        return self.encoder.predict(X)

    def analyze_reconstruction_quality(self, X_test=None):
        """
        Analyze the reconstruction quality of the autoencoder.
        
        Args:
            X_test: Optional test data. If None, uses internal data.
            
        Returns:
            dict: Dictionary with reconstruction metrics
        """
        if X_test is None:
            X_test = self.X[:100]  # Use first 100 samples for analysis
        
        # Get reconstructions
        reconstructions = self.autoencoder.predict(X_test, verbose=0)
        
        # Calculate metrics
        mse = np.mean((X_test - reconstructions) ** 2, axis=1)
        mae = np.mean(np.abs(X_test - reconstructions), axis=1)
        
        metrics = {
            'mean_mse': np.mean(mse),
            'std_mse': np.std(mse),
            'mean_mae': np.mean(mae),
            'std_mae': np.std(mae),
            'reconstruction_error_range': (np.min(mse), np.max(mse))
        }
        
        return metrics

    def save_model(self, dir, model_name='autoencoder_model.keras'):
        """
        Save the trained autoencoder model to the specified directory with the given model name.

        Args:
            dir (str):              Directory to save the model.
            model_name (str):       Name of the model file.

        Returns:
            None
        """

        self.autoencoder.save(dir + '/' + model_name)

    def load_model(self, dir, model_name):
        """
        Load the autoencoder model from the specified directory with the given model name.

        Args:
            dir (str):              Directory to load the model from.
            model_name (str):       Name of the model file.

        Returns:
            None
        """

        try:
            self.autoencoder = keras.models.load_model(dir + '/' + model_name)
        except Exception as e:
            print(f"[ERROR] Failed to load model: {e}")
            return

        self.encoder = keras.Model(inputs=self.autoencoder.input, outputs=self.autoencoder.layers[-2].output)
        print("Model loaded from", dir + '/' + model_name)