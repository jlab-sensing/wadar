from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
import matplotlib.pyplot as plt
from sklearn.model_selection import KFold
from _05_apollo import viz_tools

class SimpleRegressor:
    def __init__(self, feature_array, labels, test_size=0.2, kfold_splits=None):

        self.scaler = MinMaxScaler()
        self.features = self.scaler.fit_transform(feature_array)
        self.labels = labels
        self.test_size = test_size
        self.kfold_splits = kfold_splits

        self.X_train, self.X_test, self.y_train, self.y_test = train_test_split(
            self.features, self.labels, test_size=self.test_size, random_state=42
        )

        self.model = None
        self.history = None

    def build_model(self, input_dim, hidden_units=64):

        model = Sequential()
        model.add(Dense(hidden_units, input_dim=input_dim, activation='relu'))
        model.add(Dense(hidden_units, activation='relu'))
        model.add(Dense(1))  # Regression output
        model.compile(loss='mae', optimizer='adam')
        self.model = model

    def train(self, epochs=20, batch_size=32, validation_split=0.2):\
    
        if self.model is None:
            self.build_model(input_dim=self.X_train.shape[1])

        if not self.kfold_splits:
            self.history = self.model.fit(
                self.X_train, self.y_train,
                epochs=epochs,
                batch_size=batch_size,
                validation_split=validation_split,
                verbose=1
            )
        else:
            kf = KFold(n_splits=self.kfold_splits, shuffle=True)
            self.mae_scores = []
            for train_idx, val_idx in kf.split(self.X_train):
                X_train_fold, X_val_fold = self.X_train[train_idx], self.X_train[val_idx]
                y_train_fold, y_val_fold = self.y_train[train_idx], self.y_train[val_idx]

                self.history = self.model.fit(
                    X_train_fold, y_train_fold,
                    epochs=epochs,
                    batch_size=batch_size,
                    validation_data=(X_val_fold, y_val_fold),
                    verbose=0
                )

                y_pred_val = self.model.predict(X_val_fold).flatten()
                mae_val = np.mean(np.abs(y_val_fold - y_pred_val))
                self.mae_scores.append(mae_val)

    def evaluate(self, VIZ=False):

        y_pred = self.model.predict(self.X_test).flatten()

        if not self.kfold_splits:
            mae = np.mean(np.abs(self.y_test - y_pred))
        else:
            mae = np.mean(self.mae_scores)

        y_pred_all = self.model.predict(self.features).flatten()

        if VIZ:
            viz_tools.plot_regression(self.labels, y_pred_all)
            plt.show()
        return mae

    def predict_all(self):
        return self.model.predict(self.features).flatten()

    def evaluate_classification(self, bulk_density_to_label, VIZ = False):
        y_pred = self.predict_all()
        y_pred_labels = [bulk_density_to_label(i) for i in y_pred]
        y_true_labels = [bulk_density_to_label(i) for i in self.labels]



        print("Classification Evaluation:")
        accuracy = np.mean(np.array(y_pred_labels) == np.array(y_true_labels))
        print(f"Prediction Accuracy: {accuracy:.4f}")

        if VIZ:
            viz_tools.plot_confusion_matrix(y_true_labels, y_pred_labels)
            plt.show()
        return accuracy

