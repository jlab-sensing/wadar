"""Learned feature reduction using autoencoder."""

import logging
logger = logging.getLogger(__name__)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from sklearn.decomposition import KernelPCA
from sklearn.model_selection import ParameterGrid
from sklearn.metrics import mean_squared_error
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import Pipeline
from sklearn.model_selection import GridSearchCV
import warnings

import time
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler
from ...parameters import RANDOM_SEED

def prepare_tensor_data(X:np.ndarray, y:np.ndarray = None, batch_size:int = 256):
    """
    Convert numpy arrays to tensor data for PyTorch.

    Args:
        X (np.ndarray): Radar data to be converted to tensor form
        y (np.ndarray, optional): Labels to be converted to tensor form
        batch_size (int): Batch size for tensor dataset

    Returns:
        X_tensor (torch.Tensor):    X converted to a float32 PyTorch tensor.
        y_tensor (torch.Tensor):    y converted to a float32 PyTorch tensor.
        dataset (TensorDataset):    Dataset containing X and y tensors.
        loader (DataLoader):        DataLoader for batching and shuffling the dataset.
    """

    X_tensor = torch.tensor(X, dtype=torch.float32)
    
    if y is None:
        return X_tensor
    else:
        y_tensor = torch.tensor(y, dtype=torch.float32)
        dataset = torch.utils.data.TensorDataset(X_tensor, y_tensor)
        loader =  DataLoader(dataset=dataset, batch_size=batch_size, shuffle=True)
        return X_tensor, y_tensor, dataset, loader

class AutoencoderLearnedFeatures:
    """
    Class for learned feature reduction using an autoencoder. Much of the structure has bee sourced from
    https://github.com/rasbt/deeplearning-models/blob/master/pytorch_ipynb/autoencoder/ae-basic-with-rf.ipynb.

    Attributes:
        X (np.ndarray):                     Input feature data.
        y (np.ndarray):                     Target labels.
        epochs (int):                       Number of training epochs.
        batch_size (int):                   Batch size for training.
        verbose (bool):                     Verbosity flag for logging.
        device (torch.device):              Device used for computation (CPU or CUDA).
        scaler (MinMaxScaler):              Scaler for input normalization.
        num_features (int):                 Number of input features.
        model (torch.nn.Module):            PyTorch autoencoder model.
        optimizer (torch.optim.Optimizer):  Optimizer for training.
        dataset (TensorDataset):            PyTorch dataset for training.
        loader (DataLoader):                DataLoader for batching during training.
    """

    def __init__(self, X:np.ndarray, y:np.ndarray, epochs:int = 10, 
                 batch_size:int = 256, verbose:bool = False):
        """
        Initialize the AutoencoderLearnedFeatures class.
        
        Args:
            X (np.ndarray):     Input feature data.
            y (np.ndarray):     Target labels.
            epochs (int):       Number of training epochs. 
            batch_size (int):   Batch size for training. 
            verbose (bool):     Verbosity flag for logging. 
        """

        # Sets deterministic for reproducability
        if torch.cuda.is_available():
            torch.backends.cudnn.deterministic = True

        # Device
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        logger.info(f"Using device {self.device}")

        self.verbose = verbose
        self.epochs = epochs
        self.batch_size = batch_size

        X = self.preprocess(X)
        self.X, self.y, self.dataset, self.loader = prepare_tensor_data(X, y, batch_size=batch_size)

    def full_monty(self, X:np.ndarray):
        """
        Performs the entire dimensionality reduction process.

        Returns:
            reduced (np.ndarray):   Autoencoder-based features.
        """

        self.build_model()
        self.train()
        return self.transform(X)

    def preprocess(self, X:np.ndarray):
        """
        Preprocesses raw data and ensures that data isn't in IQ form.

        Args:
            X (np.ndarray):     Raw data

        Returns
            X (np.ndarray):     Preprocessed data
        """

        if np.iscomplex(X).any():
            logging.error("Convert the IQ data to amplitude, phase, or combine them two before using it as input.")

        X = X.reshape(X.shape[0], -1)
        self.scaler = MinMaxScaler()
        X = self.scaler.fit_transform(X)

        return X
    
    def build_model(self):
        """
        Builds autoencoder model for feature extraction.
        """

        # Hyperparameters
        learning_rate = 0.005

        # Architecture
        self.num_features = self.X.shape[1]
        num_hidden_1 = 32

        class Autoencoder(torch.nn.Module):

            def __init__(self, num_features):
                super(Autoencoder, self).__init__()
                
                ### ENCODER
                
                self.linear_1 = torch.nn.Linear(num_features, num_hidden_1)
                # The following to lones are not necessary, 
                # but used here to demonstrate how to access the weights
                # and use a different weight initialization.
                # By default, PyTorch uses Xavier/Glorot initialization, which
                # should usually be preferred.
                self.linear_1.weight.detach().normal_(0.0, 0.1)
                self.linear_1.bias.detach().zero_()
                
                ### DECODER
                self.linear_2 = torch.nn.Linear(num_hidden_1, num_features)
                self.linear_1.weight.detach().normal_(0.0, 0.1)
                self.linear_1.bias.detach().zero_()
                
            def encoder(self, x):
                encoded = self.linear_1(x)
                encoded = F.leaky_relu(encoded)
                return encoded
            
            def decoder(self, encoded_x):
                logits = self.linear_2(encoded_x)
                decoded = torch.sigmoid(logits)
                return decoded
                

            def forward(self, x):
                
                ### ENCODER
                encoded = self.encoder(x)
                
                ### DECODER
                decoded = self.decoder(encoded)
                
                return decoded
            
        torch.manual_seed(RANDOM_SEED)
        self.model = Autoencoder(num_features=self.num_features)
        self.model = self.model.to(self.device)

        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
    
    def train(self):
        """
        Trains autoencoder.
        """

        for epoch in range(self.epochs):
            for batch_idx, (features, targets) in enumerate(self.loader):
                
                # don't need labels, only the images (features)
                features = features.view(-1, self.num_features).to(self.device)
                    
                ### FORWARD AND BACK PROP
                decoded = self.model(features)
                cost = F.mse_loss(decoded, features)
                self.optimizer.zero_grad()
                
                cost.backward()
                
                ### UPDATE MODEL PARAMETERS
                self.optimizer.step()
                
                ### LOGGING
                if not batch_idx % 50:
                    if self.verbose:
                        logger.info('Epoch: %03d/%03d | Batch %03d/%03d | Cost: %.4f' 
                            %(epoch+1, self.epochs, batch_idx, 
                                len(self.loader), cost))
    
    def transform(self, X):
        """
        Apply the previously fitted autoencoder to new data.

        Args:
            X (np.ndarray):                 New input data of shape (samples, features).

        Returns:
            encoded_all (np.ndarray):       Autoencoder-based features
        """
        
        X = X.reshape(X.shape[0], -1)
        X = self.scaler.transform(X)

        self.model.eval()
        with torch.no_grad():
            encoded_all = self.model.encoder(torch.tensor(X, dtype=torch.float32).to(self.device)).cpu().numpy()

        return encoded_all