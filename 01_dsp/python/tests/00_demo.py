import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import random
from sklearn.preprocessing import MinMaxScaler

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _05_apollo import viz_tools

from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

import pandas as pd

# Set seeds for reproducibility
def set_seed(seed=42):
    random.seed(seed)
    np.random.seed(seed)
    tf.random.set_seed(seed)
    
set_seed(42)

def preprocess(X_complex):
    """Convert complex input to amplitude and phase and reshape for LSTM."""
    X_amp = np.abs(X_complex)
    X_phase = np.angle(X_complex)

    n_samples, n_range_bins, n_slow_time = X_amp.shape

    # Reshape: (samples, range_bins, slow_time) → (samples * slow_time, range_bins)
    X_amp_flat = X_amp.transpose(0, 2, 1).reshape(n_samples * n_slow_time, n_range_bins)
    X_phase_flat = X_phase.transpose(0, 2, 1).reshape(n_samples * n_slow_time, n_range_bins)

    # Stack amplitude and phase
    X_features = np.concatenate((X_amp_flat, X_phase_flat), axis=1)

    # Reshape for LSTM: (samples * slow_time, range_bins, 2)
    X_lstm = X_features.reshape(n_samples * n_slow_time, n_range_bins, 2)

    return X_lstm

def normalize_data(X_train, X_test):
    """Normalize train and test separately to prevent leakage."""
    n_timesteps, n_bins, n_features = X_train.shape
    X_train_reshaped = X_train.reshape(-1, n_bins * n_features)
    X_test_reshaped = X_test.reshape(-1, n_bins * n_features)

    scaler = MinMaxScaler()
    X_train_scaled = scaler.fit_transform(X_train_reshaped)
    X_test_scaled = scaler.transform(X_test_reshaped)

    # Reshape back
    X_train_scaled = X_train_scaled.reshape(-1, n_bins, n_features)
    X_test_scaled = X_test_scaled.reshape(-1, n_bins, n_features)

    return X_train_scaled, X_test_scaled, scaler

def build_lstm_model(input_shape):
    model = keras.Sequential([
        layers.LSTM(128, return_sequences=True, input_shape=input_shape, dropout=0.2, recurrent_dropout=0.2),
        layers.LSTM(64, return_sequences=False, dropout=0.2, recurrent_dropout=0.2),
        layers.Dense(32, activation='relu'),
        layers.Dropout(0.3),
        layers.Dense(16, activation='relu'),
        layers.Dropout(0.2),
        layers.Dense(1, activation='linear')
    ])

    model.compile(
        optimizer='adam',
        loss='mae',
        metrics=['mse', 'mae']
    )
    return model

def main():
    print("="*80)
    print("🎯 LSTM RADAR SIGNAL ANALYSIS for BULK DENSITY PREDICTION")
    print("="*80)

    # Load dataset
    print("[INFO] Loading dataset...")
    dataset_dir = "../data/training-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X_complex, y = hydros.X, hydros.y

    # Split first to avoid leakage
    X_train_c, X_test_c, y_train_base, y_test_base = train_test_split(
        X_complex, y, test_size=0.2, random_state=42
    )

    # Preprocess and expand labels
    X_train_lstm = preprocess(X_train_c)
    X_test_lstm = preprocess(X_test_c)

    n_slow_time = X_train_c.shape[2]
    y_train = np.repeat(y_train_base, n_slow_time)
    y_test = np.repeat(y_test_base, n_slow_time)

    # Normalize
    X_train_lstm, X_test_lstm, scaler = normalize_data(X_train_lstm, X_test_lstm)

    print(f"[INFO] Final LSTM input shape: {X_train_lstm.shape}")
    print(f"[INFO] y_train shape: {y_train.shape}")

    # Build model
    model = build_lstm_model((X_train_lstm.shape[1], X_train_lstm.shape[2]))
    model.summary()

    # Train
    print("\n[INFO] Training model...")
    history = model.fit(
        X_train_lstm, y_train,
        validation_data=(X_test_lstm, y_test),
        epochs=50,
        batch_size=32,
        verbose=1,
        callbacks=[
            keras.callbacks.EarlyStopping(patience=10, restore_best_weights=True),
            keras.callbacks.ReduceLROnPlateau(patience=5, factor=0.5, min_lr=1e-6)
        ]
    )

    # Predict
    y_pred = model.predict(X_test_lstm).flatten()

    # Evaluate
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)
    mae = np.mean(np.abs(y_test - y_pred))

    print("\n[INFO] Evaluation:")
    print(f"  MAE  : {mae:.6f}")
    print(f"  RMSE : {np.sqrt(mse):.6f}")
    print(f"  R²   : {r2:.6f}")

    # Save model and history
    model.save("lstm_bulk_density_model.h5")
    pd.DataFrame(history.history).to_csv("training_history.csv", index=False)

    # Plotting omitted here to keep it short — reinsert from original if needed

    print("\n[INFO] Sample Predictions:")
    print(f"{'Actual':<10} {'Predicted':<10} {'Error':<10}")
    for i in range(min(10, len(y_test))):
        actual = y_test[i]
        pred = y_pred[i]
        print(f"{actual:<10.4f} {pred:<10.4f} {abs(actual - pred):<10.4f}")

if __name__ == "__main__":
    main()


# import numpy as np
# import matplotlib.pyplot as plt
# import os
# import sys
# parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
# sys.path.insert(0, parent_dir)

# from _01_gaia.loader import FrameLoader
# from _05_apollo import viz_tools
# from scipy.signal import welch, find_peaks, hilbert
# from scipy.fft import fft, fftfreq
# from scipy.optimize import curve_fit
# from scipy.linalg import lstsq
# import pandas as pd
# import seaborn as sns

# # TensorFlow imports
# import tensorflow as tf
# from tensorflow import keras
# from tensorflow.keras import layers
# from sklearn.model_selection import train_test_split
# from sklearn.metrics import mean_squared_error, r2_score

# def main():
#     """Main analysis function for LSTM-based bulk density prediction."""
#     print("="*80)
#     print("🎯 LSTM RADAR SIGNAL ANALYSIS for BULK DENSITY PREDICTION")
#     print("="*80)
    
#     # Load the dataset
#     print("[INFO] Loading dataset...")
#     dataset_dir = "../data/training-dataset"
#     hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
#     X, y = hydros.X, hydros.y
    
#     # X is complex, y is bulk density
#     X_amplitude = np.abs(X) # (samples, range_bins, slow_time)
#     X_angle = np.angle(X) # (samples, range_bins, slow_time)
    
#     # Convert (samples, range_bins, slow_time) to (samples * slow_time, range_bins)
#     n_samples, n_range_bins, n_slow_time = X_amplitude.shape
#     X_amplitude_reshaped = X_amplitude.transpose(0, 2, 1).reshape(n_samples * n_slow_time, n_range_bins)
#     X_angle_reshaped = X_angle.transpose(0, 2, 1).reshape(n_samples * n_slow_time, n_range_bins)

#     # Concatenate the amplitude and angle features along the feature axis
#     X_features = np.concatenate((X_amplitude_reshaped, X_angle_reshaped), axis=1)

#     # Min Max normalization
#     X_features = (X_features - np.min(X_features, axis=0)) / (np.max(X_features, axis=0) - np.min(X_features, axis=0))

#     # Expand y to match the shape of X_features
#     y_expanded = np.repeat(y, n_slow_time)

#     print(y_expanded.shape, X_features.shape)

#     # Reshape data for LSTM: (samples, timesteps, features)
#     # We'll treat each range bin as a timestep and amplitude+phase as features
#     # Reshape from (samples * slow_time, range_bins * 2) to (samples * slow_time, range_bins, 2)
#     X_lstm = X_features.reshape(n_samples * n_slow_time, n_range_bins, 2)
    
#     print(f"LSTM input shape: {X_lstm.shape}")
    
#     # Split the data into training and testing sets
#     X_train, X_test, y_train, y_test = train_test_split(
#         X_lstm, y_expanded, test_size=0.2, random_state=42
#     )
    
#     print(f"Training data shape: {X_train.shape}")
#     print(f"Testing data shape: {X_test.shape}")
#     print(f"Sequence length (range bins): {X_train.shape[1]}")
#     print(f"Features per timestep: {X_train.shape[2]}")
    
#     # Build LSTM architecture
#     input_shape = (X_train.shape[1], X_train.shape[2])  # (timesteps, features)
    
#     model = keras.Sequential([
#         # First LSTM layer with return sequences for stacking
#         layers.LSTM(128, return_sequences=True, input_shape=input_shape, dropout=0.2, recurrent_dropout=0.2),
        
#         # Second LSTM layer
#         layers.LSTM(64, return_sequences=False, dropout=0.2, recurrent_dropout=0.2),
        
#         # Dense layers for regression
#         layers.Dense(32, activation='relu'),
#         layers.Dropout(0.3),
#         layers.Dense(16, activation='relu'),
#         layers.Dropout(0.2),
#         layers.Dense(1, activation='linear')  # Single output for bulk density
#     ])
    
#     # Compile the model
#     model.compile(
#         optimizer='adam',
#         loss='mae',  # Mean Absolute Error for regression
#         metrics=['mse', 'mae']
#     )
    
#     print("\nLSTM Architecture:")
#     model.summary()
    
#     # Train the LSTM
#     print("\nTraining LSTM...")
    
#     history = model.fit(
#         X_train, y_train,
#         validation_data=(X_test, y_test),
#         epochs=50,
#         batch_size=32,
#         verbose=1,
#         callbacks=[
#             keras.callbacks.EarlyStopping(
#                 patience=10, 
#                 restore_best_weights=True,
#                 monitor='val_loss'
#             ),
#             keras.callbacks.ReduceLROnPlateau(
#                 monitor='val_loss',
#                 factor=0.5,
#                 patience=5,
#                 min_lr=1e-6
#             )
#         ]
#     )
    
#     # Make predictions
#     print("\nMaking predictions...")
#     y_pred = model.predict(X_test)
    
#     # Evaluate performance
#     mse = mean_squared_error(y_test, y_pred.flatten())
#     r2 = r2_score(y_test, y_pred.flatten())
#     mae = np.mean(np.abs(y_test - y_pred.flatten()))
    
#     print(f"\n{'='*60}")
#     print("LSTM PERFORMANCE RESULTS")
#     print(f"{'='*60}")
#     print(f"Mean Squared Error (MSE): {mse:.6f}")
#     print(f"Root Mean Squared Error (RMSE): {np.sqrt(mse):.6f}")
#     print(f"Mean Absolute Error (MAE): {mae:.6f}")
#     print(f"R² Score: {r2:.6f}")
#     print(f"{'='*60}")
    
#     # Plot results
#     plt.figure(figsize=(15, 10))
    
#     # Plot 1: Training history
#     plt.subplot(2, 3, 1)
#     plt.plot(history.history['loss'], label='Training Loss')
#     plt.plot(history.history['val_loss'], label='Validation Loss')
#     plt.title('Training History (LSTM)')
#     plt.xlabel('Epoch')
#     plt.ylabel('Loss (MAE)')
#     plt.legend()
#     plt.grid(True, alpha=0.3)
    
#     # Plot 2: Predicted vs Actual
#     plt.subplot(2, 3, 2)
#     plt.scatter(y_test, y_pred.flatten(), alpha=0.6, s=20)
#     plt.plot([y_test.min(), y_test.max()], [y_test.min(), y_test.max()], 'r--', lw=2)
#     plt.xlabel('Actual Bulk Density')
#     plt.ylabel('Predicted Bulk Density')
#     plt.title(f'Predicted vs Actual (R² = {r2:.4f})')
#     plt.grid(True, alpha=0.3)
    
#     # Plot 3: Residuals
#     plt.subplot(2, 3, 3)
#     residuals = y_test - y_pred.flatten()
#     plt.scatter(y_pred.flatten(), residuals, alpha=0.6, s=20)
#     plt.axhline(y=0, color='r', linestyle='--')
#     plt.xlabel('Predicted Bulk Density')
#     plt.ylabel('Residuals')
#     plt.title('Residual Plot')
#     plt.grid(True, alpha=0.3)
    
#     # Plot 4: Sample sequence visualization
#     plt.subplot(2, 3, 4)
#     sample_idx = 0
#     plt.plot(X_test[sample_idx, :, 0], label='Amplitude', alpha=0.7)
#     plt.plot(X_test[sample_idx, :, 1], label='Phase', alpha=0.7)
#     plt.title(f'Sample Sequence\n(Actual: {y_test[sample_idx]:.3f}, Pred: {y_pred[sample_idx, 0]:.3f})')
#     plt.xlabel('Range Bin (Timestep)')
#     plt.ylabel('Normalized Value')
#     plt.legend()
#     plt.grid(True, alpha=0.3)
    
#     # Plot 5: Distribution of predictions
#     plt.subplot(2, 3, 5)
#     plt.hist(y_test, bins=20, alpha=0.7, label='Actual', density=True)
#     plt.hist(y_pred.flatten(), bins=20, alpha=0.7, label='Predicted', density=True)
#     plt.xlabel('Bulk Density')
#     plt.ylabel('Density')
#     plt.title('Distribution Comparison')
#     plt.legend()
#     plt.grid(True, alpha=0.3)
    
#     # Plot 6: Error distribution
#     plt.subplot(2, 3, 6)
#     plt.hist(residuals, bins=20, alpha=0.7, edgecolor='black')
#     plt.xlabel('Residuals (Actual - Predicted)')
#     plt.ylabel('Frequency')
#     plt.title(f'Error Distribution (MAE = {mae:.4f})')
#     plt.axvline(x=0, color='r', linestyle='--')
#     plt.grid(True, alpha=0.3)
    
#     plt.tight_layout()
#     plt.show()
    
#     # Print some sample predictions
#     print(f"\nSample Predictions:")
#     print(f"{'Actual':<10} {'Predicted':<10} {'Error':<10}")
#     print("-" * 30)
#     for i in range(min(10, len(y_test))):
#         actual = y_test[i]
#         predicted = y_pred[i, 0]
#         error = abs(actual - predicted)
#         print(f"{actual:<10.4f} {predicted:<10.4f} {error:<10.4f}")

#     # Plot a few sample sequences to visualize the data structure
#     plt.figure(figsize=(15, 8))
    
#     for i in range(6):  # Show 6 samples
#         plt.subplot(2, 3, i+1)
#         sample_idx = i * 100  # Space them out
#         if sample_idx < len(X_lstm):
#             plt.plot(X_lstm[sample_idx, :, 0], label='Amplitude', alpha=0.7)
#             plt.plot(X_lstm[sample_idx, :, 1], label='Phase', alpha=0.7)
#             plt.title(f'Sample {sample_idx}\n(Density: {y_expanded[sample_idx]:.3f})')
#             plt.xlabel('Range Bin')
#             plt.ylabel('Normalized Value')
#             if i == 0:
#                 plt.legend()
#             plt.grid(True, alpha=0.3)
    
#     plt.tight_layout()
#     plt.show()
#     plt.show()

# if __name__ == "__main__":
#     main()
