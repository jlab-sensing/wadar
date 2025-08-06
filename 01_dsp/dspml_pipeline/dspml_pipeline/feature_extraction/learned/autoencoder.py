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

def prepare_tensor_data(X, y=None, batch_size=256):
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
    PCAProcessor is a class to perform Principal Component Analysis (PCA) on radar data.

    Parameters:
        X (np.ndarray):         Input data of shape (samples, features).
        n_components (int):     Number of components to keep.
    """

    def __init__(self, X, y, epochs=10, batch_size=256, verbose=False):
        """
        Initialize the PCAProcessor.
        """

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

    def full_monty(self, X):
        self.build_model()
        self.train()
        return self.transform(X)

    def preprocess(self, X):

        if np.iscomplex(X).any():
            logging.error("Convert the IQ data to amplitude, phase, or combine them two before using it as input.")

        X = X.reshape(X.shape[0], -1)
        self.scaler = MinMaxScaler()
        X = self.scaler.fit_transform(X)

        return X
    
    def build_model(self):

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
        start_time = time.time()
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
        
        X = X.reshape(X.shape[0], -1)
        X = self.scaler.transform(X)

        self.model.eval()
        with torch.no_grad():
            encoded_all = self.model.encoder(torch.tensor(X, dtype=torch.float32).to(self.device)).cpu().numpy()

        return encoded_all