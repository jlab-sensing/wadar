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

if __name__ == "__main__":

    # load the dataset
    dataset_dir = "../data/combined-soil-compaction-dataset"
    feature_file_name = "features_selected.csv"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y

    # flatten (scan x fast time (range bins) x slow time (pulses)) to (scan x fast time)
    print(X.shape)
    new_X = np.zeros((X.shape[0] * X.shape[2], X.shape[1]), dtype=complex)
    for i in range(X.shape[0]):
        for j in range(X.shape[2]):
            new_X[i * X.shape[2] + j, :] = X[i, :, j]
    new_Y = np.repeat(y, X.shape[2])
    X = new_X
    y = new_Y

    # https://bobrupakroy.medium.com/lstms-for-regression-cc9b6677697f

    # assemble dataset matrix
    def create_dataset(dataset, labels, look_back=1):
        dataX, dataY = [], []
        for i in range(len(dataset) - look_back):
            a = dataset[i:(i + look_back), :]
            dataX.append(a)
            dataY.append(labels[i + look_back])
        return np.array(dataX), np.array(dataY)

    # random seed
    np.random.seed(44)

    # Scale X only
    scaler_X = MinMaxScaler()
    dataset_scaled = scaler_X.fit_transform(np.abs(X))

    # Scale Y separately if you want (or leave raw)
    scaler_y = MinMaxScaler()
    y_scaled = scaler_y.fit_transform(y.reshape(-1, 1))

    # Train-test split on scaled data
    X_train, X_test, y_train, y_test = train_test_split(dataset_scaled, y_scaled, test_size=0.2)

    # Create LSTM datasets
    look_back = 10
    trainX, trainY = create_dataset(X_train, y_train, look_back)
    testX, testY = create_dataset(X_test, y_test, look_back)

    # No reshaping needed if you use create_dataset() correctly
    # Shape will be (samples, look_back, features)

    # Define model
    model = Sequential()
    model.add(LSTM(15, input_shape=(look_back, trainX.shape[2])))
    model.add(Dense(1))
    model.compile(loss='mean_squared_error', optimizer='adam',metrics =["accuracy"])
    model.fit(trainX, trainY, epochs=30, batch_size=28, verbose=1)

    # Predictions
    trainPredict = model.predict(trainX)
    testPredict = model.predict(testX)

    # Invert scaling on Y
    trainPredict = scaler_y.inverse_transform(trainPredict)
    trainY = scaler_y.inverse_transform(trainY)
    testPredict = scaler_y.inverse_transform(testPredict)
    testY = scaler_y.inverse_transform(testY)

    # Evaluate
    trainScore = np.sqrt(mean_squared_error(trainY, trainPredict))
    testScore = np.sqrt(mean_squared_error(testY, testPredict))

    print(f'Train Score: {trainScore:.2f} RMSE')
    print(f'Test Score: {testScore:.2f} RMSE')

    # Plot accuracy

    plt.figure()
    plt.plot(model.history.history['accuracy'], label='train accuracy')
    plt.plot(model.history.history['val_accuracy'], label='test accuracy')
    plt.title('Model Accuracy')
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend()
    plt.show()