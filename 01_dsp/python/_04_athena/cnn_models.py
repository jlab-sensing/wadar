from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split, KFold
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, Flatten, Dense
from _05_apollo import viz_tools
from _06_hermes.parameters import num2label, RANDOM_SEED
import tensorflow as tf
import time
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from _06_hermes.parameters import KFOLD_SPLITS

tf.random.set_seed(RANDOM_SEED)

class CNN1D:
    def __init__(self, X, y, n_splits=KFOLD_SPLITS):
        self.X = self.scale_each_sample(X)
        self.y = y
        self.n_splits = n_splits
        
    def scale_each_sample(self, X):
        X_scaled = np.zeros_like(X)
        for i in range(X.shape[0]):
            X_scaled[i] = MinMaxScaler().fit_transform(X[i])
        return X_scaled
    
    def full_monty(self, epochs=50, batch_size=32):

        self.model = self.build_model(input_shape=(self.X.shape[1], self.X.shape[2]))
        metrics = self.cross_validate(epochs=epochs)
        model = self.train(epochs=epochs, batch_size=batch_size)

        self.model = model 

        return model, metrics

    def build_model(self, input_shape, filters=64, dense_units=64):
        model = Sequential([
            Conv1D(filters, 3, activation='relu', input_shape=input_shape),
            Conv1D(filters, 3, activation='relu'),
            Flatten(),
            Dense(dense_units, activation='relu'),
            Dense(1)
        ])
        model.compile(optimizer='adam', loss='mae')
        return model

    def train(self, epochs=50, batch_size=32):

        self.model = self.build_model(input_shape=(self.X.shape[1], self.X.shape[2]))
        self.model.fit(self.X, self.y, epochs=epochs, batch_size=batch_size, verbose=1)

        return self.model
    
    def cross_validate(self, epochs):
        kf = KFold(n_splits=self.n_splits, shuffle=True, random_state=RANDOM_SEED)

        mae = []
        inference_time = []
        accuracy = []
        rmse = []
        training_time = []
        r2 = []

        for train_index, val_index in kf.split(self.X):
            X_train, X_val = self.X[train_index], self.X[val_index]
            y_train, y_val = self.y[train_index], self.y[val_index]


            model = self.build_model(input_shape=(X_train.shape[1], X_train.shape[2]))
            
            time_start = time.time()
            model.fit(X_train, y_train, epochs=epochs, batch_size=32, verbose=1,
                        validation_data=(X_val, y_val))
            training_time.append(time.time() - time_start)

            time_start = time.time()            # done seperately just to measure inference time
            y_pred = model.predict(X_val)
            inference_time.append(time.time() - time_start)

            mae.append(mean_absolute_error(y_val, y_pred))
            rmse.append(np.sqrt(mean_squared_error(y_val, y_pred)))
            r2.append(r2_score(y_val, y_pred))

            y_labels = [num2label(label) for label in y_val]
            accuracy.append(np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)]))

        metrics = {
            'mae': np.mean(mae),
            'inference_time': np.mean(inference_time),
            'accuracy': np.mean(accuracy),
            'rmse': np.mean(rmse),
            'training_time': np.mean(training_time),
            'r2': np.mean(r2)
        }

        return metrics

    def estimate(self, X):
        X_scaled = self.scale_each_sample(X)
        y_pred = self.model.predict(X_scaled).flatten()
        return y_pred

    def evaluate(self, VIZ=True):

        X = self.X_val
        y = self.y_val

        time_start = time.time()
        X = self.scale_each_sample(X)
        y_pred = self.estimate(X)
        inference_time = time.time() - time_start

        mae = mean_absolute_error(y, y_pred)

        y_labels = [num2label(label) for label in y]
        accuracy = np.mean([num2label(pred) == y for pred, y in zip(y_pred, y_labels)])

        if VIZ:
            viz_tools.plot_regression(y, y_pred)
            plt.show()
        
        return {
            'mae': mae,
            'accuracy': accuracy,
            'inference_time': inference_time
        }

    def save_model(self, model_dir, model_name="baby_cnn_regressor.keras"):
        self.model.save(model_dir + '/' + model_name)

    def load_model(self, model_dir, model_name="baby_cnn_regressor.keras"):
        self.model = tf.keras.models.load_model(model_dir + '/' + model_name)
        return self.model
    
    def predict(self, X):
        X_scaled = self.scale_each_sample(X)
        y_pred = self.model.predict(X_scaled).flatten()
        return y_pred