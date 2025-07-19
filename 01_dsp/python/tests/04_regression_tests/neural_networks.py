import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from sklearn.metrics import mean_squared_error
import pandas as pd
from keras.layers import Dense, Conv1D, Flatten, Dropout
from sklearn.model_selection import KFold
from _06_hermes.bulk_density_labels import bulk_density_to_label
from _05_apollo import viz_tools
from _03_hephaestus import feature_tools
import tensorflow as tf

tf.get_logger().setLevel('ERROR')

if __name__ == "__main__":

    VIZ = False  # Set to True to visualize features
    
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"
    test_size = 0.2

    # If making a new feature set,
    # hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    # X, y = hydros.X, hydros.y
    # hephaestus_features = feature_tools.FeatureTools(X)
    # feature_table = hephaestus_features.feature_full_monty(y, dataset_dir)

    # If using an existing feature set,
    feature_table, _, _, _ = feature_tools.load_feature_table(
        dataset_dir, feature_file_name)
    
    df_best, mi_scores = feature_tools.mutual_info_minimize_features(feature_table, top_n=10)
    feature_tools.save_feature_table(df_best, dataset_dir, "features_mutual_info.csv")
    _, feature_array, feature_names, labels = feature_tools.load_feature_table(
        dataset_dir, "features_mutual_info.csv")

    # ===================================================

    # scaler = MinMaxScaler()
    # X_scaled = scaler.fit_transform(feature_array)

    # X_train, X_test, y_train, y_test = train_test_split(
    #     X_scaled, labels, test_size=0.2
    # )

    # # simple lil test model
    # model = Sequential()
    # model.add(Dense(64, input_dim=X_train.shape[1], activation='relu'))
    # model.add(Dense(64, activation='relu'))
    # model.add(Dense(1))  # regression output

    # model.compile(loss='mse', optimizer='adam')

    # history = model.fit(X_train, y_train, epochs=20, batch_size=32,
    #                     validation_split=0.2, verbose=1)


    # y_pred = model.predict(X_test).flatten()
    # mse = mean_squared_error(y_test, y_pred)
    # print(f"Test MSE: {mse:.4f}")

    # y_pred = model.predict(X_scaled).flatten()

    # y_pred_labels = [bulk_density_to_label(i) for i in y_pred]
    # y_test_labels = [bulk_density_to_label(i) for i in labels]

    # accuracy = np.mean(np.array(y_pred_labels) == np.array(y_test_labels))
    # print(f"Prediction Accuracy: {accuracy:.4f}")

    # viz_tools.plot_confusion_matrix(
    #     y_test_labels, y_pred_labels
    # )

   # ===================================================

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X = np.abs(hydros.X)
    y = hydros.y

    X_scaled = np.zeros_like(X)
    for i in range(X.shape[0]):
        X_scaled[i] = MinMaxScaler().fit_transform(X[i])

    X_main, X_holdout, y_main, y_holdout = train_test_split(
        X_scaled, y, test_size=0.2, random_state=42
    )

    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    mae_scores = []

    for train_idx, val_idx in kf.split(X_main):
        X_train, X_val = X_main[train_idx], X_main[val_idx]
        y_train, y_val = y_main[train_idx], y_main[val_idx]

        model = Sequential([        # baby's first CNN
            Conv1D(64, 3, activation='relu', input_shape=(X_train.shape[1], X_train.shape[2])),
            Conv1D(64, 3, activation='relu'),
            Flatten(),
            Dense(64, activation='relu'),
            Dense(1)
        ])
        model.compile(optimizer='adam', loss='mae')

        model.fit(X_train, y_train, epochs=50, batch_size=32,
                validation_split=0.1, verbose=1)

        y_pred_val = model.predict(X_val).flatten()
        mae_val = np.mean(np.abs(y_val - y_pred_val))
        mae_scores.append(mae_val)

    print(f"K-Fold Validation MAEs: {mae_scores}")
    print(f"Average Validation MAE: {np.mean(mae_scores):.4f}")

    final_model = Sequential([
        Conv1D(64, 3, activation='relu', input_shape=(X_main.shape[1], X_main.shape[2])),
        Conv1D(64, 3, activation='relu'),
        Flatten(),
        Dense(64, activation='relu'),
        Dense(1)
    ])
    final_model.compile(optimizer='adam', loss='mae')

    final_model.fit(X_main, y_main, epochs=50, batch_size=32, validation_split=0.1, verbose=0)

    y_pred_holdout = final_model.predict(X_holdout).flatten()
    mae_holdout = np.mean(np.abs(y_holdout - y_pred_holdout))

    print(f"Hold-out Test MAE: {mae_holdout:.4f}")

    all_y_pred = final_model.predict(X_scaled).flatten()

    plt.figure(figsize=(10, 6))
    plt.plot(y, label='True Values', alpha=0.5)
    plt.plot(all_y_pred, label='Predicted Values', alpha=0.5)
    plt.title('True vs Predicted Values')
    plt.xlabel('Sample Index')
    plt.ylabel('Value')
    plt.legend()
    plt.show()