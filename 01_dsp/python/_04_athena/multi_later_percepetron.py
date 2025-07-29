from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
import numpy as np
import time
import pickle
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Input, Dropout
from _06_hermes.parameters import num2label, RANDOM_SEED, KFOLD_SPLITS
from tensorflow.keras.models import load_model

np.random.seed(RANDOM_SEED)

class MultiLayerPerceptron:
    """
    Multi-Layer Perceptron for regression.
    """
    
    def __init__(self, feature_array, labels, kfold_splits=KFOLD_SPLITS):
        """
        Initialize MLP with feature scaling.
        
        Parameters:
            feature_array: Raw features to be scaled and used for training
            labels: Target labels
            kfold_splits: Number of folds for cross-validation
        """

        self.scaler = MinMaxScaler()
        self.raw_features = feature_array  # Keep original for reference
        self.features = self.scaler.fit_transform(feature_array)  # Scaled features
        self.labels = labels
        self.kfold_splits = kfold_splits
        self.model = None
        self.history = None

    def full_monty(self, epochs=100, batch_size=32):
        """
        Complete training pipeline: cross-validation + final model training.
        
        Parameters:
            epochs: Number of training epochs
            batch_size: Training batch size
            
        Returns:
            Tuple of (trained_model, cross_validation_metrics)
        """

        self._build_model(input_dim=self.features.shape[1])
        metrics = self._cross_validate(epochs=epochs, batch_size=batch_size)
        self.model = self._train_final_model(epochs=epochs, batch_size=batch_size)
        
        return self.model, metrics

    def _build_model(self, input_dim, hidden_units_1=128, hidden_units_2=256, dropout_rate=0.5):
        """
        Build the neural network architecture.
        
        Parameters:
            input_dim: Number of input features
            hidden_units_1: Units in first hidden layer
            hidden_units_2: Units in second hidden layer  
            dropout_rate: Dropout rate for regularization
        """

        model = Sequential([
            Input(shape=(input_dim,)),
            Dense(hidden_units_1, activation='relu'),
            Dropout(dropout_rate),
            Dense(hidden_units_2, activation='relu'),
            Dropout(dropout_rate),
            Dense(1)  # Regression output
        ])
        
        model.compile(optimizer='adam', loss='mean_squared_error', metrics=['mae'])
        self.model = model
        return model

    def _cross_validate(self, epochs=20, batch_size=32):
        """
        Perform k-fold cross-validation.
        
        Parameters:
            epochs: Number of training epochs per fold
            batch_size: Training batch size
            
        Returns:
            Dictionary of averaged metrics across folds
        """

        kf = KFold(n_splits=self.kfold_splits, shuffle=True, random_state=RANDOM_SEED)
        
        metrics = {
            'mae': [], 'rmse': [], 'r2': [], 'accuracy': [],
            'training_time': [], 'inference_time': []
        }

        for train_idx, val_idx in kf.split(self.features):
            # Build fresh model for each fold
            self._build_model(input_dim=self.features.shape[1])

            X_train_fold, X_val_fold = self.features[train_idx], self.features[val_idx]
            y_train_fold, y_val_fold = self.labels[train_idx], self.labels[val_idx]

            # Train with timing
            start_time = time.time()
            self.model.fit(
                X_train_fold, y_train_fold,
                epochs=epochs,
                batch_size=batch_size,
                validation_data=(X_val_fold, y_val_fold),
                verbose=0
            )
            training_time = time.time() - start_time

            # Predict with timing
            inference_start = time.time()
            y_pred_val = self.model.predict(X_val_fold, verbose=0).flatten()
            inference_time = time.time() - inference_start

            # Calculate metrics
            mae = mean_absolute_error(y_val_fold, y_pred_val)
            rmse = np.sqrt(mean_squared_error(y_val_fold, y_pred_val))
            r2 = r2_score(y_val_fold, y_pred_val)
            
            # Calculate accuracy using num2label
            y_labels = [num2label(label) for label in y_val_fold]
            y_pred_labels = [num2label(pred) for pred in y_pred_val]
            accuracy = np.mean([pred == true for pred, true in zip(y_pred_labels, y_labels)])

            # Store metrics
            metrics['mae'].append(mae)
            metrics['rmse'].append(rmse)
            metrics['r2'].append(r2)
            metrics['accuracy'].append(accuracy)
            metrics['training_time'].append(training_time)
            metrics['inference_time'].append(inference_time)

        # Return averaged metrics
        return {key: np.mean(values) for key, values in metrics.items()}

    def _train_final_model(self, epochs=20, batch_size=32):
        """
        Train final model on full dataset.
        
        Parameters:
            epochs: Number of training epochs
            batch_size: Training batch size
            
        Returns:
            Trained model
        """

        self._build_model(input_dim=self.features.shape[1])
        self.model.fit(
            self.features, self.labels,
            epochs=epochs,
            batch_size=batch_size,
            verbose=0
        )
        return self.model

    def predict(self, new_features):
        """
        Make predictions on new data with proper scaling.
        
        Parameters:
            new_features: Raw feature array to predict on
            
        Returns:
            Predictions array
            
        Raises:
            ValueError: If model hasn't been trained yet
        """

        if self.model is None:
            raise ValueError("Model must be trained before making predictions. Call full_monty() first.")
        
        # Apply same scaling as training data
        scaled_features = self.scaler.transform(new_features)
        
        # Make predictions
        predictions = self.model.predict(scaled_features, verbose=0).flatten()
        
        return predictions
    
    def save_model(self, dataset_dir, name="MLP"):
        """
        Save both the trained model and scaler.
        
        Parameters:
            dataset_dir: Directory to save files
            name: Base name for saved files
            
        Raises:
            ValueError: If model hasn't been trained yet
        """

        if self.model is None:
            raise ValueError("Model must be trained before saving. Call full_monty() first.")
            
        model_path = f"{dataset_dir}/{name}.h5"
        scaler_path = f"{dataset_dir}/{name}_scaler.pkl"

        # Save model
        self.model.save(model_path)
        
        # Save scaler
        with open(scaler_path, 'wb') as f:
            pickle.dump(self.scaler, f)
            
        print(f"Model saved to: {model_path}")
        print(f"Scaler saved to: {scaler_path}")

    @classmethod
    def load_model(cls, dataset_dir, name="MLP"):
        """
        Load a saved model and scaler.
        
        Parameters:
            dataset_dir: Directory containing saved files
            name: Base name of saved files
            
        Returns:
            Loaded MLP instance with model and scaler restored
        """
        
        model_path = f"{dataset_dir}/{name}.h5"
        scaler_path = f"{dataset_dir}/{name}_scaler.pkl"
        
        # Create dummy instance
        dummy_features = np.array([[0]])  # Will be replaced
        dummy_labels = np.array([0])
        instance = cls(dummy_features, dummy_labels)
        
        # Load model and scaler
        instance.model = load_model(model_path)
        with open(scaler_path, 'rb') as f:
            instance.scaler = pickle.load(f)
            
        return instance