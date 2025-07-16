import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
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

if __name__ == "__main__":

    # load the dataset
    dataset_dir = "../data/dry-soil-compaction-dataset"
    feature_file_name = "features_selected.csv"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    data = pd.read_csv(f"{dataset_dir}/{feature_file_name}")
    X = data.drop(columns=['label']).values
    y = data['label'].values


    # scaler = MinMaxScaler()
    # X_scaled = scaler.fit_transform(X)

    # X_train, X_test, y_train, y_test = train_test_split(
    #     X_scaled, y, test_size=0.2
    # )

    # # simple lil test model
    # model = Sequential()
    # model.add(Dense(64, input_dim=X_train.shape[1], activation='relu'))
    # model.add(Dense(64, activation='relu'))
    # model.add(Dense(1))  # regression output

    # model.compile(loss='mse', optimizer='adam')

    # history = model.fit(X_train, y_train, epochs=100, batch_size=32,
    #                     validation_split=0.2, verbose=1)


    # y_pred = model.predict(X_test).flatten()
    # mse = mean_squared_error(y_test, y_pred)
    # print(f"Test MSE: {mse:.4f}")

    X = hydros.X   # samples, fast_time, slow_time)
    y = hydros.y
    X_abs = np.abs(X)

    # Normalize per sample
    scaler = MinMaxScaler()
    X_scaled = np.zeros_like(X_abs)
    for i in range(X_abs.shape[0]):
        X_scaled[i] = scaler.fit_transform(X_abs[i])

    # Train/test split without flattening
    X_train, X_test, y_train, y_test = train_test_split(
        X_scaled, y, test_size=0.2, random_state=42
    )

    # # Build model — Conv1D along slow_time axis (time distributed over fast_time)
    # model = Sequential()
    # model.add(Conv1D(64, kernel_size=3, activation='relu', input_shape=(X_train.shape[1], X_train.shape[2])))
    # model.add(Conv1D(64, kernel_size=3, activation='relu'))
    # model.add(Flatten())
    # model.add(Dense(64, activation='relu'))
    # model.add(Dense(1))

    # model.compile(loss='mse', optimizer='adam')
    # history = model.fit(
    #     X_train, y_train, epochs=100, batch_size=32,
    #     validation_split=0.2, verbose=1
    # )

    # y_pred = model.predict(X_test).flatten()
    # mse = mean_squared_error(y_test, y_pred)
    # print(f"Test MSE: {mse:.4f}")

    # print(X_train.shape, X_test.shape)

    kf = KFold(n_splits=5, shuffle=True)
    mse_scores = []

    for train_idx, test_idx in kf.split(X_scaled):
        X_train, X_test = X_scaled[train_idx], X_scaled[test_idx]
        y_train, y_test = y[train_idx], y[test_idx]

        model = Sequential()
        model.add(Conv1D(64, 3, activation='relu', input_shape=(X_train.shape[1], X_train.shape[2])))
        model.add(Conv1D(64, 3, activation='relu'))
        model.add(Flatten())
        model.add(Dense(64, activation='relu'))
        model.add(Dense(1))
        model.compile(optimizer='adam', loss='mse')
        model.fit(X_train, y_train, epochs=50, batch_size=16, verbose=0)

        y_pred = model.predict(X_test).flatten()
        mse = mean_squared_error(y_test, y_pred)
        mse_scores.append(mse)

    print(f"K-Fold MSEs: {mse_scores}")
    print(f"Average MSE: {np.mean(mse_scores):.4f}")