import logging
logger = logging.getLogger(__name__)

import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import time
import os
from PIL import Image

from ..parameters import RANDOM_SEED, KFOLD_SPLITS, num2label
from transformers import MobileViTFeatureExtractor, MobileViTForImageClassification

# Set seeds for reproducibility
torch.manual_seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)

class RadarData2Image(Dataset):
    """
    Dataset class to convert radar data to images for MobileViT processing.
    """
    
    def __init__(self, X, y, feature_extractor):
        """
        Initialize the dataset.
        
        Parameters:
            X: Radar data of shape (N, H, W) 
            y: Labels corresponding to the radar data
            feature_extractor: MobileViT feature extractor
        """
        self.X = np.abs(X)
        self.y = y
        self.feature_extractor = feature_extractor      

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
        Get an item from the dataset.

        Parameters:
            idx: Index of the item to retrieve
            
        Returns:
            Tuple containing processed image inputs and corresponding label
        """

        # Get the sample and ensure it's properly normalized
        sample = self.X[idx].copy()
        
        # Check for NaN/Inf and handle
        if np.any(np.isnan(sample)) or np.any(np.isinf(sample)):
            sample = np.nan_to_num(sample, nan=0.0, posinf=1.0, neginf=0.0)
        
        # Normalize to [0, 1] range first
        sample_min = sample.min()
        sample_max = sample.max()
        
        if sample_max > sample_min:
            sample = (sample - sample_min) / (sample_max - sample_min)
        else:
            sample = np.zeros_like(sample)
        
        # Now convert to [0, 255] range for image
        img = Image.fromarray((sample * 255).astype(np.uint8)).convert("RGB")
        inputs = self.feature_extractor(images=img, return_tensors="pt")
        inputs = {k: v.squeeze(0) for k, v in inputs.items()}
        
        # Handle label
        label_val = self.y[idx]
        if np.isnan(label_val) or np.isinf(label_val):
            label_val = 0.0
            
        label = torch.tensor(label_val, dtype=torch.float32)
        return inputs, label
    
class TransformerEstimator(nn.Module):
    """
    Transformer-based regression model for radar signal analysis using MobileViT.
    """
    
    def __init__(self, X, y, epochs=10, batch_size=4, verbose=False):

        super().__init__()

        # Set device to GPU if available, otherwise CPU
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.kfold_splits = KFOLD_SPLITS
        self.epochs = epochs
        self.batch_size = batch_size
        self.verbose = verbose

        # Store raw data for cross-validation
        self.X_raw = X
        self.y = y

        # Load MobileViT model
        self.mobilevit = MobileViTForImageClassification.from_pretrained("apple/mobilevit-small")
        
        # Get the actual classifier input size by looking at the existing classifier
        original_classifier = self.mobilevit.classifier
        actual_hidden_size = original_classifier.in_features
        
        # Replace the classifier with our regression head
        self.mobilevit.classifier = nn.Linear(actual_hidden_size, 1)
        
        # Initialize the new classifier
        nn.init.xavier_uniform_(self.mobilevit.classifier.weight)
        nn.init.zeros_(self.mobilevit.classifier.bias)
        
        # Move to device
        self.mobilevit.to(self.device)

        self.feature_extractor = MobileViTFeatureExtractor.from_pretrained("apple/mobilevit-small")
        
        # Freeze the backbone, only train the classifier
        for name, param in self.mobilevit.named_parameters():
            if 'classifier' not in name:
                param.requires_grad = False

    def full_monty(self):
        """
        Complete transformer evaluation pipeline: data processing + cross-validation.
        """

        logger.info(f"Processing complex radar data for Transformer...")

        # Perform cross-validation
        metrics = self.cross_validate()
        
        # Train final model
        model = self.train_final_model()
                                            
        return model, metrics

    def forward(self, x):
        """
        Forward pass through the MobileViT model.
        
        Args:
            x (torch.Tensor): Input images tensor
            
        Returns:
            torch.Tensor: Regression outputs
        """
        outputs = self.mobilevit(x)
        return outputs.logits

    def _train(self, X, y):
        """
        Train the model on provided data.
        
        Args:
            X: Input radar data
            y: Target values
            
        Returns:
            Trained model
        """
        
        dataset = RadarData2Image(X, y, self.feature_extractor)
        dataloader = DataLoader(dataset, batch_size=self.batch_size, shuffle=True)

        optimizer = torch.optim.AdamW(self.parameters(), lr=1e-4)
        loss_fn = nn.L1Loss()

        for epoch in range(self.epochs):
            train_loss = train(self, dataloader, optimizer, loss_fn, self.device)
            if self.verbose:
                logger.info(f"Epoch {epoch+1}/{self.epochs}: Train Loss = {train_loss:.4f}")

        return self

    def train_final_model(self):
        """
        Train the final model on all available data.
        
        Returns:
            Trained model
        """
        
        # Reset classifier weights for final training
        actual_hidden_size = self.mobilevit.classifier.in_features
        self.mobilevit.classifier = nn.Linear(actual_hidden_size, 1).to(self.device)
        nn.init.xavier_uniform_(self.mobilevit.classifier.weight)
        nn.init.zeros_(self.mobilevit.classifier.bias)
        
        # Train on all data
        return self._train(self.X_raw, self.y)

    def cross_validate(self):
        """
        Perform k-fold cross-validation with proper normalization to prevent data leakage.
        """

        kfold = KFold(n_splits=self.kfold_splits, shuffle=True, random_state=RANDOM_SEED)
        
        cv_mse_scores = []
        cv_mae_scores = []
        cv_rmse_scores = []
        cv_accuracy_scores = []
        fold_times = []
        fold_training_times = []
        fold_inference_times = []
        
        logger.info(f"Starting {self.kfold_splits}-fold cross-validation...")

        for fold, (train_idx, val_idx) in enumerate(kfold.split(self.X_raw)):
            fold_start_time = time.time()
            
            # Split data
            X_train_fold = self.X_raw[train_idx]
            X_val_fold = self.X_raw[val_idx]
            y_train_fold = self.y[train_idx]
            y_val_fold = self.y[val_idx]

            # Create datasets
            train_dataset = RadarData2Image(X_train_fold, y_train_fold, self.feature_extractor)
            val_dataset = RadarData2Image(X_val_fold, y_val_fold, self.feature_extractor)
            train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True)
            val_loader = DataLoader(val_dataset, batch_size=self.batch_size)

            # Reset classifier weights for each fold to avoid accumulation issues
            actual_hidden_size = self.mobilevit.classifier.in_features
            self.mobilevit.classifier = nn.Linear(actual_hidden_size, 1).to(self.device)
            nn.init.xavier_uniform_(self.mobilevit.classifier.weight)
            nn.init.zeros_(self.mobilevit.classifier.bias)

            optimizer = torch.optim.AdamW(self.parameters(), lr=1e-4)
            loss_fn = nn.L1Loss()

            # Track training time
            training_start_time = time.time()
            for epoch in range(self.epochs):
                train_loss = train(self, train_loader, optimizer, loss_fn, self.device)
                if self.verbose:
                    logger.info(f"Fold {fold+1}, Epoch {epoch+1}/{self.epochs}: Train Loss = {train_loss:.4f}")
            training_time = time.time() - training_start_time
            fold_training_times.append(training_time)

            # Track inference time
            inference_start_time = time.time()
            val_loss, y_pred_fold, y_val_fold = evaluate(self, val_loader, loss_fn, self.device)
            inference_time = time.time() - inference_start_time
            fold_inference_times.append(inference_time)

            # Calculate metrics
            mse = mean_squared_error(y_val_fold, y_pred_fold)
            mae = mean_absolute_error(y_val_fold, y_pred_fold)
            rmse = np.sqrt(mse)
            
            # Calculate accuracy using num2label
            y_labels = [num2label(label) for label in y_val_fold]
            y_pred_labels = [num2label(pred) for pred in y_pred_fold]
            accuracy = np.mean([pred_label == true_label for pred_label, true_label in zip(y_pred_labels, y_labels)])
            
            cv_mse_scores.append(mse)
            cv_mae_scores.append(mae)
            cv_rmse_scores.append(rmse)
            cv_accuracy_scores.append(accuracy)
            
            fold_time = time.time() - fold_start_time
            fold_times.append(fold_time)
            
            logger.info(f"Fold {fold+1}/{self.kfold_splits} - MAE: {mae:.2f}, RMSE: {rmse:.2f}, "
                        f"Accuracy: {100*accuracy:.2f}%, Training time: {1000*training_time:.2f}ms, "
                        f"Inference time: {1000*inference_time:.2f}ms")

        # Calculate average metrics
        metrics = {
            'mae': np.mean(cv_mae_scores),
            'mse': np.mean(cv_mse_scores),
            'rmse': np.mean(cv_rmse_scores),
            'accuracy': np.mean(cv_accuracy_scores),
            'training_time': np.mean(fold_training_times),
            'inference_time': np.mean(fold_inference_times),
            'total_time': np.sum(fold_times)
        }

        logger.info(f"Average metrics - MAE: {metrics['mae']:.2f}, RMSE: {metrics['rmse']:.2f}, "
                   f"Accuracy: {100*metrics['accuracy']:.2f}%, Training time: {1000*metrics['training_time']:.2f}ms, "
                   f"Inference time: {1000*metrics['inference_time']:.2f}ms")
        
        return metrics

    def forward(self, pixel_values):
        """
        Forward pass through the MobileViT model.

        Args:
            pixel_values (torch.Tensor): Input tensor of shape [batch_size, channels, height, width].
        
        Returns:
            torch.Tensor: Predicted regression outputs of shape [batch_size].
        """
        outputs = self.mobilevit(pixel_values)
        return outputs.logits.squeeze(-1)  # [B, 1] -> [B]

    def estimate(self, X_complex):
        """
        Make predictions on new data.
        
        Parameters:
            X_complex: Complex radar data
            
        Returns:
            Predictions
        """

        if self.mobilevit is None:
            raise ValueError("Model not trained. Call train_final_model first.")

        self.mobilevit.to(self.device)
        self.mobilevit.eval()

        dataset = RadarData2Image(X_complex, np.zeros(len(X_complex)), self.feature_extractor)
        dataloader = DataLoader(dataset, batch_size=self.batch_size)

        preds = []
        with torch.no_grad():
            for batch in dataloader:
                inputs, _ = batch
                pixel_values = inputs['pixel_values'].to(self.device)
                outputs = self(pixel_values)
                preds.extend(outputs.cpu().numpy())
        return np.array(preds)
    
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
    model.mobilevit.to(device)
    total_loss = 0
    
    for batch in dataloader:
        inputs, targets = batch
        pixel_values = inputs['pixel_values'].to(device)
        targets = targets.to(device)

        optimizer.zero_grad()
        outputs = model(pixel_values)
        loss = loss_fn(outputs, targets)
        
        if torch.isnan(loss):
            logger.warning("NaN loss detected during training")
            return float('nan')
            
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
        
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
    
    model.eval()
    model.mobilevit.to(device)
    total_loss = 0
    preds, trues = [], []
    with torch.no_grad():
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