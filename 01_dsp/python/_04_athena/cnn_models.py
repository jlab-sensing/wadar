from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split, KFold
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, Flatten, Dense
from _05_apollo import viz_tools

class BabyCNNRegressor:
    def __init__(self, X, y, test_size=0.2, n_splits=5):
        self.X_scaled = self.scale_each_sample(X)
        self.y = y
        self.test_size = test_size
        self.n_splits = n_splits

        self.X_main, self.X_holdout, self.y_main, self.y_holdout = train_test_split(
            self.X_scaled, self.y, test_size=self.test_size, random_state=42
        )

        self.final_model = None

    def scale_each_sample(self, X):
        X_scaled = np.zeros_like(X)
        for i in range(X.shape[0]):
            X_scaled[i] = MinMaxScaler().fit_transform(X[i])
        return X_scaled

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

    def cross_validate(self, epochs=50, batch_size=32):
        kf = KFold(n_splits=self.n_splits, shuffle=True, random_state=42)
        mae_scores = []

        for train_idx, val_idx in kf.split(self.X_main):
            X_train, X_val = self.X_main[train_idx], self.X_main[val_idx]
            y_train, y_val = self.y_main[train_idx], self.y_main[val_idx]

            model = self.build_model(input_shape=(X_train.shape[1], X_train.shape[2]))
            model.fit(X_train, y_train, epochs=epochs, batch_size=batch_size,
                      validation_split=0.1, verbose=0)

            y_pred_val = model.predict(X_val).flatten()
            mae_val = np.mean(np.abs(y_val - y_pred_val))
            mae_scores.append(mae_val)

        print(f"K-Fold Validation MAEs: {mae_scores}")
        print(f"Average Validation MAE: {np.mean(mae_scores):.4f}")
        return mae_scores

    def train_final_model(self, epochs=50, batch_size=32):
        self.final_model = self.build_model(
            input_shape=(self.X_main.shape[1], self.X_main.shape[2])
        )
        self.final_model.fit(self.X_main, self.y_main, epochs=epochs,
                             batch_size=batch_size, validation_split=0.1, verbose=0)

    def evaluate_holdout(self):
        y_pred_holdout = self.final_model.predict(self.X_holdout).flatten()
        mae_holdout = np.mean(np.abs(self.y_holdout - y_pred_holdout))
        print(f"Hold-out Test MAE: {mae_holdout:.4f}")
        return mae_holdout

    def plot_predictions(self):
        all_y_pred = self.final_model.predict(self.X_scaled).flatten()

        viz_tools.plot_regression(self.y, all_y_pred)
        plt.show()

