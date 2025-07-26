import numpy as np
import matplotlib.pyplot as plt
import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
sys.path.insert(0, parent_dir)

import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
import requests
from PIL import Image
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from _06_hermes.parameters import KFOLD_SPLITS, RANDOM_SEED, num2label
from sklearn.model_selection import KFold
import time

class RadarData2Image(Dataset):
    def __init__(self, X, y, processor):
        """
        Dataset class to convert radar data to images for BLIP processing.

        Args:
            X (np.ndarray):             Radar data of shape (N, H, W) where N is the number of samples, H is height, and W is width.
            y (np.ndarray):             Labels corresponding to the radar data.
            processor (BlipProcessor):  Processor for BLIP that formats input for the model.
        """

        self.X = X
        self.y = y
        self.processor = processor      

    def __len__(self):
        """
        Lets len(dataset) return the number of samples in the dataset.

        Args:
            None

        Returns:
            int: Number of samples in the dataset.
        """

        return len(self.X)

    def __getitem__(self, idx):
        """
        Gets an item from the dataset.

        Args:
            idx (int): Index of the item to retrieve.

        Returns:
            tuple: A tuple containing the processed image inputs and the corresponding label.
            - inputs (dict): Processed image inputs ready for the BLIP model.
            - label (torch.Tensor): Corresponding label as a PyTorch tensor.
        """

        img = Image.fromarray((self.X[idx] * 255).astype(np.uint8)).convert("RGB")      # Converts radar data at idx into an 8-bit RGB image.
        inputs = self.processor(images=img, return_tensors="pt")
        inputs = {k: v.squeeze(0) for k, v in inputs.items()}                           # Turns image into tensor and removes the batch dimension.
        label = torch.tensor(self.y[idx], dtype=torch.float32)                          # Prepares the label as a PyTorch float tensor.
        return inputs, label
    
class TransformerRegressionHead(nn.Module):
    def __init__(self, encoder, output_dim=1024):
        """
        Regression head that uses a transformer encoder (like BLIP) to extract features and predict a single regression output.

        Args:
            encoder (nn.Module):       Pretrained transformer encoder (e.g., BLIP).
            output_dim (int):          Dimensionality of the encoder's output features.
        """

        super().__init__()
        self.encoder = encoder.vision_model             # Only using to vision encoder because the other parts of the model are for captioning.
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.regressor = nn.Linear(output_dim, 1)       # Encoder's output is averaged (pooled) and run through a linear layer to produce a single regression output.

        for param in self.encoder.parameters():
            param.requires_grad = False                 # Freezes the encoder's parameters to prevent them from being updated during training.

    def train_full(self, X, y, processor, epochs=10, batch_size=4, save_dir=None):
        """
        Trains the regression model on the provided radar data. TODO: Verify that this works.
        
        Args:
            X (np.ndarray):             Radar data of shape (N, H, W).
            y (np.ndarray):             Labels corresponding to the radar data.
            processor (BlipProcessor):  Processor for BLIP that formats input for the model.
            epochs (int):               Number of training epochs.
            batch_size (int):           Batch size for training.
            save_dir (str):             Directory to save the trained model.
                
        Returns:
            self (nn.Module):           The trained regression model.
        """

        # Set device to GPU if available, otherwise CPU
        device = "cuda" if torch.cuda.is_available() else "cpu"

        dataset = RadarData2Image(X, y, processor)
        dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

        optimizer = torch.optim.AdamW(self.parameters(), lr=1e-4)
        loss_fn = nn.L1Loss()

        for epoch in range(epochs):
            train_loss = train(self, dataloader, optimizer, loss_fn, device)
            print(f"Epoch {epoch}: Train Loss = {train_loss:.4f}")
        
        if save_dir:
            torch.save(self.state_dict(), save_dir)

        return self
    
    def load_model(self, model_path):
        """
        Loads a pre-trained model from the specified path.

        Args:
            model_path (str): Path to the saved model file.

        Returns:
            None
        """

        self.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
        self.eval()

    def predict(self, X, processor, batch_size=4, device=None):
        """
        Predicts regression outputs for the given radar data.

        Args:
            X (np.ndarray):             Radar data of shape (N, H, W).
            processor (BlipProcessor):  Processor for BLIP that formats input for the model.
            batch_size (int):           Batch size for prediction.
            device (str, optional):     Device to run the model on ('cuda' or 'cpu'). If None, auto-detect.

        Returns:
            np.ndarray: Predicted regression outputs.
        """
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.to(device)
        self.eval()

        dataset = RadarData2Image(X, np.zeros(len(X)), processor)
        dataloader = DataLoader(dataset, batch_size=batch_size)

        preds = []
        with torch.no_grad():
            for batch in dataloader:
                inputs, _ = batch
                pixel_values = inputs['pixel_values'].to(device)
                outputs = self(pixel_values)
                preds.extend(outputs.cpu().numpy())
        return np.array(preds)

    def full_monty_eval(self, X, y, processor, epochs=10, batch_size=4):
        """
        Full training and evaluation pipeline for the model.

        Args:
            X (np.ndarray):             Radar data of shape (N, H, W).
            y (np.ndarray):             Labels corresponding to the radar data.
            processor (BlipProcessor):  Processor for BLIP that formats input for the model.

        Returns:
            dict: A dictionary containing evaluation metrics such as MAE, accuracy, RMSE, R2 score, inference time, and training time.
        """

        # Set device to GPU if available, otherwise CPU
        device = "cuda" if torch.cuda.is_available() else "cpu"

        kf = KFold(n_splits=KFOLD_SPLITS, shuffle=True, random_state=RANDOM_SEED)
        
        mae = []
        accuracy = []
        rmse = []
        inference_times = []
        training_times = []
        r2 = []

        for fold, (train_idx, val_idx) in enumerate(kf.split(X)):
            
            X_train, X_val = X[train_idx], X[val_idx]
            y_train, y_val = y[train_idx], y[val_idx]

            train_dataset = RadarData2Image(X_train, y_train, processor)
            val_dataset = RadarData2Image(X_val, y_val, processor)
            train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
            val_loader = DataLoader(val_dataset, batch_size=batch_size)

            optimizer = torch.optim.AdamW(self.parameters(), lr=1e-4)
            loss_fn = nn.L1Loss()

            print(f"Fold {fold+1}")
            start_time = time.time()
            for epoch in range(epochs):
                train_loss = train(self, train_loader, optimizer, loss_fn, device)
                start_time_2 = time.time()
                val_loss, preds, trues = evaluate(self, val_loader, loss_fn, device)
                inference_time = time.time() - start_time_2
                print(f"  Epoch {epoch}: Train Loss = {train_loss:.4f}, Val Loss = {val_loss:.4f}, Time Passed = {time.time() - start_time:.2f}s")

            training_times.append(time.time() - start_time)
            inference_times.append(inference_time)

            mae.append(mean_absolute_error(trues, preds))
            y_labels = [num2label(label) for label in trues]
            accuracy.append(np.mean([num2label(pred) == y for pred, y in zip(preds, y_labels)]))
            rmse.append(np.sqrt(mean_squared_error(trues, preds)))
            r2.append(r2_score(trues, preds))

        return {
            "mae": np.mean(mae),
            "accuracy": np.mean(accuracy),
            "rmse": np.mean(rmse),
            "r2": np.mean(r2),
            "inference_time": np.mean(inference_times),
            "training_time": np.mean(training_times)
        }

    def forward(self, pixel_values):
        """
        Forward pass through the regression head.

        Args:
            pixel_values (torch.Tensor): Input tensor of shape [batch_size, channels, height, width].
        
        Returns:
            torch.Tensor: Predicted regression outputs of shape [batch_size].
        """

        outputs = self.encoder(pixel_values=pixel_values)
        last_hidden_state = outputs.last_hidden_state                       # [batch, patch_count, hidden_dim]
        pooled = self.pool(last_hidden_state.transpose(1, 2)).squeeze(-1)   # [B, hidden_dim] -> [B, 1]
        return self.regressor(pooled).squeeze(-1)                           # [B, 1] -> [B]

def train(model, dataloader, optimizer, loss_fn, device):
    """
    Standard training loop for the regression model.

    Args:
        model (nn.Module):                  The regression model to train.
        dataloader (DataLoader):            DataLoader providing batches of data.
        optimizer (torch.optim.Optimizer):  Optimizer for updating model weights.
        loss_fn (nn.Module):                Loss function to compute the error.
        device (str):                       Device to run the model on ('cuda' or 'cpu').

    Returns:
        float: Average loss over the epoch.
    """

    model.train()
    total_loss = 0
    for batch in dataloader:
        inputs, targets = batch                                 # Inputs is a dictionary from BLIP?
        pixel_values = inputs['pixel_values'].to(device)
        targets = targets.to(device)                            # Move inputs and targets to the specified device (GPU/CPU)

        optimizer.zero_grad()                                   # Unlike Keras, PyTorch's process seems to be more manual.
        outputs = model(pixel_values)                           # Forward pass
        loss = loss_fn(outputs, targets)                        # Compute loss
        loss.backward()                                         # Backward propagation
        optimizer.step()                                        # Update weights

        total_loss += loss.item()                               # Average loss over each epoch 
    return total_loss / len(dataloader)

def evaluate(model, dataloader, loss_fn, device):
    """
    Evaluates the model on the validation set.

    Args:
        model (nn.Module):         The regression model to evaluate.
        dataloader (DataLoader):   DataLoader providing batches of data.
        loss_fn (nn.Module):       Loss function to compute the error.
        device (str):              Device to run the model on ('cuda' or 'cpu').
    Returns:
        float: Average loss over the validation set.
        list: Predicted values.
        list: True values.
    """
    
    model.eval()                                                # Disables dropout because evaluation is deterministic.
    total_loss = 0
    preds, trues = [], []
    with torch.no_grad():                                       # To save memory?
        for batch in dataloader:
            inputs, targets = batch
            pixel_values = inputs['pixel_values'].to(device)
            targets = targets.to(device)

            outputs = model(pixel_values)
            loss = loss_fn(outputs, targets)

            total_loss += loss.item()
            preds.extend(outputs.cpu().numpy())
            trues.extend(targets.cpu().numpy())
    return total_loss / len(dataloader), preds, trues