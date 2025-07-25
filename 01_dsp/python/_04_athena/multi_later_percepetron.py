from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
import matplotlib.pyplot as plt
from sklearn.model_selection import KFold
from _05_apollo import viz_tools
from _06_hermes.parameters import KFOLD_SPLITS
from tensorflow.keras.layers import Dropout
from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
import time
from _06_hermes.parameters import num2label, RANDOM_SEED
from _03_hephaestus import feature_tools

np.random.seed(RANDOM_SEED)

class MultiLaterPercepetron:
    def __init__(self, feature_array, labels, kfold_splits=KFOLD_SPLITS):

        self.scaler = MinMaxScaler()
        self.features = self.scaler.fit_transform(feature_array)
        self.labels = labels
        self.kfold_splits = kfold_splits

        self.model = None
        self.history = None

    def full_monty(self, epochs=100, batch_size=32):
        
        self.build_model(input_dim=self.features.shape[1])
        metrics = self.cross_validate(epochs=epochs, batch_size=batch_size)
        self.model = self.train(epochs=epochs, batch_size=batch_size)
        
        return self.model, metrics

    def build_model(self, input_dim):

        hidden_units_1=128
        hidden_units_2=256
        dropout_rate=0.5

        model = Sequential()
        model.add(Dense(hidden_units_1, input_dim=input_dim, activation='relu'))
        model.add(Dropout(dropout_rate))
        model.add(Dense(hidden_units_2, activation='relu'))
        model.add(Dropout(dropout_rate))
        model.add(Dense(1))  # Regression output
        model.compile(loss='mae', optimizer='adam')
        self.model = model

    def cross_validate(self, epochs=20, batch_size=32):

        kf = KFold(n_splits=self.kfold_splits, shuffle=True)

        mae_scores = []
        rmse_scores = []
        r2_scores = []
        training_times = []
        inference_times = []
        accuracy_scores = []
        

        for train_idx, val_idx in kf.split(self.features):

            self.build_model(input_dim=self.features.shape[1])

            X_train_fold, X_val_fold = self.features[train_idx], self.features[val_idx]
            y_train_fold, y_val_fold = self.labels[train_idx], self.labels[val_idx]

            start_time = time.time()
            self.history = self.model.fit(
                X_train_fold, y_train_fold,
                epochs=epochs,
                batch_size=batch_size,
                validation_data=(X_val_fold, y_val_fold),
                verbose=0
            )
            training_time = time.time() - start_time

            inference_start_time = time.time()
            y_pred_val = self.model.predict(X_val_fold).flatten()
            inference_time = time.time() - inference_start_time

            mae = mean_absolute_error(y_val_fold, y_pred_val)
            rmse = np.sqrt(mean_squared_error(y_val_fold, y_pred_val))
            r2 = r2_score(y_val_fold, y_pred_val)

            mae_scores.append(mae)
            rmse_scores.append(rmse)
            r2_scores.append(r2)
            training_times.append(training_time)
            inference_times.append(inference_time)

            y_labels = [num2label(label) for label in y_val_fold]
            fold_accuracy = np.mean([num2label(pred) == y_true for pred, y_true in zip(y_pred_val, y_labels)])
            accuracy_scores.append(fold_accuracy)

        return {"mae": np.mean(mae_scores), "rmse": np.mean(rmse_scores), "r2": np.mean(r2_scores), "training_time": np.mean(training_times), "inference_time": np.mean(inference_times), "accuracy": np.mean(accuracy_scores)}

    def train(self, epochs=20, batch_size=32):
        self.model.fit(
                self.features, self.labels,
                epochs=epochs,
                batch_size=batch_size,
                verbose=0
            )
        
        return self.model
    
    def save_model(self, model, dataset_dir):
        model_path = f"{dataset_dir}/models/MLP.h5"
        model.save(model_path)

def monte_carlo_mlp_feature_selection(feature_table, labels, data_dir, n_iterations=100):
    """
    Perform Monte Carlo feature selection for Multi-Layer Perceptron regression.
    """
    feature_array = feature_table.values
    num_features = feature_array.shape[1]
    
    best_features = []
    best_score = float('inf')

    for _ in range(n_iterations):
        selected_features = np.random.choice(num_features, size=int(num_features * 0.5), replace=False)
        X_selected = feature_array[:, selected_features]

        model = MultiLaterPercepetron(X_selected, labels)
        metrics = model.cross_validate()

        if metrics['mae'] < best_score:
            best_score = metrics['mae']
            best_features = selected_features
            print(f"Current iteration MAE: {metrics['mae']:.4f} at iteration {_ + 1}")

    feature_array_optimal = feature_array[:, best_features]
    feature_table_optimal = feature_table.iloc[:, best_features].copy()
    feature_table_optimal['Label'] = labels

    feature_tools.save_feature_table(
        feature_table_optimal, data_dir, "models/feature_mlp_monte_carlo.csv"
    )

    return feature_array_optimal, feature_table_optimal.columns.tolist(), labels