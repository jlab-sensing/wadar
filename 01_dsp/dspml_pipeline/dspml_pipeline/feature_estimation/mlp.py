import logging
logger = logging.getLogger(__name__)

import time
import sys
from sklearn.model_selection import KFold
from sklearn.metrics import mean_absolute_error, mean_squared_error
import copy
import numpy as np
import torch
import tqdm
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler

from ..parameters import KFOLD_SPLITS, RANDOM_SEED, num2label, GRID_SEARCH_SCORING

class MLPRegression:
    def __init__(self):
        self.model = None
        self.metrics = None

    def full_monty(self, feature_array, labels):
        metrics = self.cross_validate(feature_array=feature_array, labels=labels)

        feature_array = self.scale_input(feature_array)
        feature_array = self.preprocess_input(feature_array)
        labels = self.preprocess_output(labels)
        model = self.train(feature_array, labels)
        return model, metrics
    
    def scale_input(self, X_train, X_test=None):
        self.scaler = StandardScaler()
        self.scaler.fit(X_train)
        X_train = self.scaler.transform(X_train)
        if X_test is None:
            return X_train
        X_test = self.scaler.transform(X_test)

        X_train = self.preprocess_input(X_train)
        X_test = self.preprocess_input(X_test)

        return X_train, X_test
    
    def preprocess_input(self, X):
        return torch.tensor(X, dtype=torch.float32)
    
    def preprocess_output(self, y):
        return torch.tensor(y, dtype=torch.float32).reshape(-1, 1)

    def build_model(self, X):
        super().__init__()
        # Define the model
        input_size = X.shape[1]
        output_size = 1 # Continuous float
        self.model = nn.Sequential(             # TODO: Play around with this.
            nn.Linear(input_size, 128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(128, 256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, 1)
        )

        # Loss function and optimizer
        self.loss_fn = nn.MSELoss()  # mean square error
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.001)

    def cross_validate(self, feature_array, labels):
        X, y = feature_array, labels
        kf = KFold(n_splits=KFOLD_SPLITS, shuffle=True, random_state=RANDOM_SEED)

        # Store metrics for each fold
        metrics = {'mae': [], 'rmse': [], 'accuracy': [], 'inference_time': [], 'training_time': []}

        # Cross-validation loop
        fold_num = 0
        for train_index, test_index in kf.split(X):
            
            fold_num += 1
            X_train, X_test = X[train_index], X[test_index]
            y_train, y_test = y[train_index], y[test_index]

            X_train, X_test = self.scale_input(X_train, X_test)
            y_train, y_test = self.preprocess_output(y_train), self.preprocess_output(y_test)

            # Time training
            train_start = time.time()
            model = self.train(X_train, y_train, X_test, y_test)
            training_time = time.time() - train_start

            # Time inference
            inference_start = time.time()
            y_pred = self.model(X_test).detach().numpy()
            inference_time = time.time() - inference_start

            # Calculate metrics
            mae = mean_absolute_error(y_test, y_pred)
            rmse = np.sqrt(mean_squared_error(y_test, y_pred))
            
            # Calculate accuracy using num2label
            y_test_labels = [num2label(label) for label in y_test]
            y_pred_labels = [num2label(pred) for pred in y_pred]
            accuracy = np.mean([pred == true for pred, true in zip(y_pred_labels, y_test_labels)])

            # Store metrics
            metrics['mae'].append(mae)
            metrics['rmse'].append(rmse)
            metrics['accuracy'].append(accuracy)
            metrics['inference_time'].append(inference_time)
            metrics['training_time'].append(training_time)

            logger.info(f"Fold {fold_num}/{KFOLD_SPLITS} - MAE: {mae:.2f}, RMSE: {rmse:.2f}, "
                        f"Accuracy: {100*accuracy:.2f}%, Training time: {1000*training_time:.2f}ms, "
                        f"Inference time: {1000*inference_time:.2f}ms")

        # Store averaged metrics
        metrics = {
            "mae": np.mean(metrics['mae']),
            "rmse": np.mean(metrics['rmse']),
            "accuracy": np.mean(metrics['accuracy']),
            "inference_time": np.mean(metrics['inference_time']),
            "training_time": np.mean(metrics['training_time'])
        }

        logger.info(f"Average metrics - MAE: {metrics['mae']:.2f}, RMSE: {metrics['rmse']:.2f}, "
                   f"Accuracy: {100*metrics['accuracy']:.2f}%, Training time: {1000*metrics['training_time']:.2f}ms, "
                   f"Inference time: {1000*metrics['inference_time']:.2f}ms")

        return metrics
    
    def train(self, X_train, y_train, X_test=None, y_test=None):

        self.build_model(X_train)

        # Training parameters
        n_epochs = 1000   # number of epochs to run
        batch_size = 32  # size of each batch
        batch_start = torch.arange(0, len(X_train), batch_size)

        # Hold the best model
        best_mse = np.inf   # init to infinity
        best_weights = None
        history = []

        # If not testing data, just train on training data
        if X_test is None:
            for epoch in range(n_epochs):
                self.model.train()
                for start in batch_start:
                    X_batch = X_train[start:start+batch_size]
                    y_batch = y_train[start:start+batch_size]

                    y_pred = self.model(X_batch)
                    loss = self.loss_fn(y_pred, y_batch)
                    self.optimizer.zero_grad()
                    loss.backward()
                    self.optimizer.step()
        else:
            # training loop
            for epoch in range(n_epochs):
                self.model.train()
                with tqdm.tqdm(batch_start, unit="batch", mininterval=0, disable=True) as bar:
                    bar.set_description(f"Epoch {epoch}")
                    for start in bar:
                        # take a batch
                        X_batch = X_train[start:start+batch_size]
                        y_batch = y_train[start:start+batch_size]
                        # forward pass
                        y_pred = self.model(X_batch)
                        loss = self.loss_fn(y_pred, y_batch)
                        # backward pass
                        self.optimizer.zero_grad()
                        loss.backward()
                        # update weights
                        self.optimizer.step()
                        # print progress
                        bar.set_postfix(mse=float(loss))
                # evaluate accuracy at end of each epoch
                self.model.eval()
                y_pred = self.model(X_test)
                mse = self.loss_fn(y_pred, y_test)
                mse = float(mse)
                history.append(mse)
                if mse < best_mse:
                    best_mse = mse
                    best_weights = copy.deepcopy(self.model.state_dict())

            # restore model and return best accuracy
            self.model.load_state_dict(best_weights)

        return self.model

    def estimate(self, X):
        if self.model is None:
            logger.error("Model has not been fitted yet. Call train() first.")
            sys.exit(1)
        X = self.scaler.transform(X)
        X = self.preprocess_input(X)
        self.model.eval()
        with torch.no_grad():
            return self.model(X).numpy().flatten()
