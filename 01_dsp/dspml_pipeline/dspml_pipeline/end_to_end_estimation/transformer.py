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
from transformers import BlipProcessor, BlipForConditionalGeneration

# Set seeds for reproducibility
torch.manual_seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)


class RadarData2Image(Dataset):
    """
    Dataset class to convert radar data to images for transformer processing.
    """
    
    def __init__(self, X, y, processor):
        """
        Initialize dataset.
        
        Parameters:
            X: Radar data of shape (N, H, W)
            y: Labels corresponding to the radar data
            processor: Processor for transformer that formats input for the model
        """
        self.X = X
        self.y = y
        self.processor = processor

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        """
        Get an item from the dataset.
        
        Returns:
            tuple: Processed image inputs and corresponding label
        """
        # Convert radar data to RGB image
        img = Image.fromarray((self.X[idx] * 255).astype(np.uint8)).convert("RGB")
        inputs = self.processor(images=img, return_tensors="pt")
        inputs = {k: v.squeeze(0) for k, v in inputs.items()}
        label = torch.tensor(self.y[idx], dtype=torch.float32)
        return inputs, label


class TransformerRegressionHead(nn.Module):
    """
    Regression head using transformer encoder for feature extraction.
    """
    
    def __init__(self, encoder, output_dim=1024):
        """
        Initialize regression head.
        
        Parameters:
            encoder: Pretrained transformer encoder
            output_dim: Dimensionality of encoder's output features
        """
        super().__init__()
        self.encoder = encoder.vision_model
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.regressor = nn.Linear(output_dim, 1)
        
        # Freeze encoder parameters
        for param in self.encoder.parameters():
            param.requires_grad = False

    def forward(self, pixel_values):
        """
        Forward pass through the regression head.
        
        Parameters:
            pixel_values: Input tensor of shape [batch_size, channels, height, width]
            
        Returns:
            Predicted regression outputs of shape [batch_size]
        """
        outputs = self.encoder(pixel_values=pixel_values)
        last_hidden_state = outputs.last_hidden_state
        pooled = self.pool(last_hidden_state.transpose(1, 2)).squeeze(-1)
        return self.regressor(pooled).squeeze(-1)


class TransformerEstimator:
    """
    Transformer-based regression model for radar signal analysis.
    """
    
    def __init__(self, X, y, output_dim=1024, batch_size=4, epochs=10, verbose=0):
        """
        Initialize Transformer regressor.

        Parameters:
            X: Complex radar data
            y: Target values
            output_dim: Encoder output dimensionality
            batch_size: Batch size for training
            epochs: Number of training epochs
            verbose: Verbosity level
        """

        self.kfold_splits = KFOLD_SPLITS
        self.model = None
        self.history = None

        # Set device
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # Hard code encoder and processor
        self.processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-large")
        blip = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-large").to(self.device)
        self.encoder = blip
        self.output_dim = blip.vision_model.config.hidden_size

        self.batch_size = batch_size
        self.epochs = epochs
        self.verbose = verbose

        # Store raw data for cross-validation
        self.X_raw = X
        self.y = y

    def full_monty(self):
        """
        Complete Transformer evaluation pipeline: data processing + cross-validation.
        """
        
        logger.info(f"Processing complex radar data for Transformer...")
        
        # Perform cross-validation
        metrics = self.cross_validate()
        
        # Train final model on all data
        self.model = self.train_final_model()
        
        return self.model, metrics

    def _process_complex_data(self, X):
        """
        Convert complex radar data to normalized amplitude images with better preprocessing.
        
        Parameters:
            X: Complex radar data of shape (samples, range_bins, slow_time)
            
        Returns:
            Normalized amplitude data ready for image conversion
        """
        # Convert complex data to amplitude
        amplitude = np.abs(X)
        
        # Normalize each sample individually to [0, 1] range with better numerical stability
        normalized_data = []
        for sample in amplitude:
            # Add small epsilon to prevent division by zero
            sample_min = sample.min()
            sample_max = sample.max()
            
            # Handle edge case where all values are the same
            if sample_max - sample_min < 1e-8:
                normalized_sample = np.ones_like(sample) * 0.5  # Set to middle value
            else:
                normalized_sample = (sample - sample_min) / (sample_max - sample_min)
            
            # Clip to ensure values are in [0, 1]
            normalized_sample = np.clip(normalized_sample, 0.0, 1.0)
            
            # Check for NaN/inf
            if np.isnan(normalized_sample).any() or np.isinf(normalized_sample).any():
                logger.warning("NaN/Inf detected in normalized sample, replacing with zeros")
                normalized_sample = np.zeros_like(sample)
            
            normalized_data.append(normalized_sample)
            
        return np.stack(normalized_data)

    def build_model(self):
        """
        Build transformer regression model with proper initialization.
        """
        self.model = TransformerRegressionHead(self.encoder, self.output_dim)
        self.model.to(self.device)
        
        # Initialize the regression layer properly
        with torch.no_grad():
            self.model.regressor.weight.normal_(0, 0.01)
            self.model.regressor.bias.fill_(0)
        
        return self.model

    def _train_epoch(self, model, dataloader, optimizer, loss_fn):
        """
        Train model for one epoch with NaN protection.
        """
        model.train()
        total_loss = 0
        valid_batches = 0
        
        for batch_idx, batch in enumerate(dataloader):
            try:
                inputs, targets = batch
                pixel_values = inputs['pixel_values'].to(self.device)
                targets = targets.to(self.device)
                
                # Check for NaN/inf in inputs
                if torch.isnan(pixel_values).any() or torch.isinf(pixel_values).any():
                    logger.warning(f"NaN/Inf detected in inputs at batch {batch_idx}")
                    continue
                    
                if torch.isnan(targets).any() or torch.isinf(targets).any():
                    logger.warning(f"NaN/Inf detected in targets at batch {batch_idx}")
                    continue
                
                optimizer.zero_grad()
                
                # Forward pass with gradient checking
                outputs = model(pixel_values)
                
                # Check for NaN/inf in outputs
                if torch.isnan(outputs).any() or torch.isinf(outputs).any():
                    logger.warning(f"NaN/Inf detected in model outputs at batch {batch_idx}")
                    continue
                
                loss = loss_fn(outputs, targets)
                
                # Check for NaN/inf in loss
                if torch.isnan(loss) or torch.isinf(loss):
                    logger.warning(f"NaN/Inf loss detected at batch {batch_idx}, skipping")
                    continue
                
                loss.backward()
                
                # Gradient clipping to prevent exploding gradients
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                
                # Check for NaN gradients
                has_nan_grad = False
                for param in model.parameters():
                    if param.grad is not None:
                        if torch.isnan(param.grad).any() or torch.isinf(param.grad).any():
                            has_nan_grad = True
                            break
                
                if has_nan_grad:
                    logger.warning(f"NaN/Inf gradients detected at batch {batch_idx}, skipping optimizer step")
                    continue
                
                optimizer.step()
                
                total_loss += loss.item()
                valid_batches += 1
                
            except Exception as e:
                logger.warning(f"Error in batch {batch_idx}: {e}")
                continue
        
        if valid_batches == 0:
            logger.error("No valid batches processed!")
            return float('inf')
            
        return total_loss / valid_batches

    def _evaluate_epoch(self, model, dataloader, loss_fn):
        """
        Evaluate model on validation set with NaN protection.
        """
        model.eval()
        total_loss = 0
        preds, trues = [], []
        valid_batches = 0
        
        with torch.no_grad():
            for batch_idx, batch in enumerate(dataloader):
                try:
                    inputs, targets = batch
                    pixel_values = inputs['pixel_values'].to(self.device)
                    targets = targets.to(self.device)
                    
                    # Check for NaN/inf in inputs
                    if torch.isnan(pixel_values).any() or torch.isinf(pixel_values).any():
                        logger.warning(f"NaN/Inf detected in validation inputs at batch {batch_idx}")
                        continue
                        
                    if torch.isnan(targets).any() or torch.isinf(targets).any():
                        logger.warning(f"NaN/Inf detected in validation targets at batch {batch_idx}")
                        continue
                    
                    outputs = model(pixel_values)
                    
                    # Check for NaN/inf in outputs
                    if torch.isnan(outputs).any() or torch.isinf(outputs).any():
                        logger.warning(f"NaN/Inf detected in validation outputs at batch {batch_idx}")
                        continue
                    
                    loss = loss_fn(outputs, targets)
                    
                    # Check for NaN/inf in loss
                    if torch.isnan(loss) or torch.isinf(loss):
                        logger.warning(f"NaN/Inf validation loss detected at batch {batch_idx}")
                        continue
                    
                    total_loss += loss.item()
                    preds.extend(outputs.cpu().numpy())
                    trues.extend(targets.cpu().numpy())
                    valid_batches += 1
                    
                except Exception as e:
                    logger.warning(f"Error in validation batch {batch_idx}: {e}")
                    continue
        
        if valid_batches == 0:
            logger.error("No valid validation batches processed!")
            return float('inf'), np.array([]), np.array([])
            
        return total_loss / valid_batches, np.array(preds), np.array(trues)

    def cross_validate(self):
        """
        Perform k-fold cross-validation.
        """
        
        # Process complex data to normalized amplitude
        X_processed = self._process_complex_data(self.X_raw)
        
        kfold = KFold(n_splits=self.kfold_splits, shuffle=True, random_state=RANDOM_SEED)
        
        cv_mse_scores = []
        cv_mae_scores = []
        cv_rmse_scores = []
        cv_accuracy_scores = []
        cv_r2_scores = []
        fold_times = []
        fold_training_times = []
        fold_inference_times = []
        
        logger.info(f"Starting {self.kfold_splits}-fold cross-validation...")
        
        for fold, (train_idx, val_idx) in enumerate(kfold.split(X_processed)):
            fold_start_time = time.time()
            
            # Split data
            X_train_fold = X_processed[train_idx]
            X_val_fold = X_processed[val_idx]
            y_train_fold = self.y[train_idx]
            y_val_fold = self.y[val_idx]
            
            # Create datasets and dataloaders
            train_dataset = RadarData2Image(X_train_fold, y_train_fold, self.processor)
            val_dataset = RadarData2Image(X_val_fold, y_val_fold, self.processor)
            train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True)
            val_loader = DataLoader(val_dataset, batch_size=self.batch_size)
            
            # Build and train model
            model = self.build_model()
            optimizer = torch.optim.AdamW(
                model.parameters(), 
                lr=1e-5,  # Lower learning rate
                weight_decay=1e-4,  # Add weight decay for regularization
                eps=1e-8  # Epsilon for numerical stability
            )
            
            # Add learning rate scheduler
            scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                optimizer, mode='min', factor=0.5, patience=3
            )
            
            loss_fn = nn.MSELoss()  # Change to MSE for better stability
            
            # Track training time
            training_start_time = time.time()

            best_val_loss = float('inf')
            patience_counter = 0
            max_patience = 5
            
            for epoch in range(self.epochs):
                train_loss = self._train_epoch(model, train_loader, optimizer, loss_fn)
                
                # Early stopping check
                if np.isnan(train_loss) or np.isinf(train_loss):
                    logger.error(f"Training failed due to NaN/Inf loss at epoch {epoch+1}")
                    break
                
                # Validation check for learning rate scheduling
                val_loss, _, _ = self._evaluate_epoch(model, val_loader, loss_fn)
                scheduler.step(val_loss)
                
                if val_loss < best_val_loss:
                    best_val_loss = val_loss
                    patience_counter = 0
                else:
                    patience_counter += 1
                
                if patience_counter >= max_patience:
                    logger.info(f"Early stopping at epoch {epoch+1}")
                    break
                
                if self.verbose > 0:
                    print(f"Fold {fold+1}/{self.kfold_splits}, Epoch {epoch+1}/{self.epochs}: Train Loss = {train_loss:.4f}, Val Loss = {val_loss:.4f}")
            
            training_time = time.time() - training_start_time
            fold_training_times.append(training_time)
            
            # Track inference time and get predictions
            inference_start_time = time.time()
            val_loss, y_pred_fold, y_true_fold = self._evaluate_epoch(model, val_loader, loss_fn)
            inference_time = time.time() - inference_start_time
            fold_inference_times.append(inference_time)
            
            # Calculate metrics
            mse = mean_squared_error(y_true_fold, y_pred_fold)
            mae = mean_absolute_error(y_true_fold, y_pred_fold)
            rmse = np.sqrt(mse)
            r2 = r2_score(y_true_fold, y_pred_fold)
            
            # Calculate accuracy using num2label
            y_labels = [num2label(label) for label in y_true_fold]
            y_pred_labels = [num2label(pred) for pred in y_pred_fold]
            accuracy = np.mean([pred_label == true_label for pred_label, true_label in zip(y_pred_labels, y_labels)])
            
            cv_mse_scores.append(mse)
            cv_mae_scores.append(mae)
            cv_rmse_scores.append(rmse)
            cv_r2_scores.append(r2)
            cv_accuracy_scores.append(accuracy)
            
            fold_time = time.time() - fold_start_time
            fold_times.append(fold_time)
            
            logger.info(f"Fold {fold+1}/{self.kfold_splits} - MAE: {mae:.2f}, RMSE: {rmse:.2f}, "
                        f"R2: {r2:.3f}, Accuracy: {100*accuracy:.2f}%, Training time: {1000*training_time:.2f}ms, "
                        f"Inference time: {1000*inference_time:.2f}ms")
        
        # Calculate average metrics
        metrics = {
            'mae': np.mean(cv_mae_scores),
            'mse': np.mean(cv_mse_scores),
            'rmse': np.mean(cv_rmse_scores),
            'r2': np.mean(cv_r2_scores),
            'accuracy': np.mean(cv_accuracy_scores),
            'training_time': np.mean(fold_training_times),
            'inference_time': np.mean(fold_inference_times),
            'total_time': np.sum(fold_times)
        }
        
        logger.info(f"Average metrics - MAE: {metrics['mae']:.2f}, RMSE: {metrics['rmse']:.2f}, "
                   f"R2: {metrics['r2']:.3f}, Accuracy: {100*metrics['accuracy']:.2f}%, "
                   f"Training time: {1000*metrics['training_time']:.2f}ms, "
                   f"Inference time: {1000*metrics['inference_time']:.2f}ms")
        
        return metrics

    def train_final_model(self):
        """
        Train the final model on all data.
        """
        
        # Process complex data to normalized amplitude
        X_processed = self._process_complex_data(self.X_raw)
        
        # Create dataset and dataloader
        dataset = RadarData2Image(X_processed, self.y, self.processor)
        dataloader = DataLoader(dataset, batch_size=self.batch_size, shuffle=True)
        
        # Build and train final model
        self.model = self.build_model()
        optimizer = torch.optim.AdamW(
            self.model.parameters(), 
            lr=1e-5,  # Lower learning rate
            weight_decay=1e-4,
            eps=1e-8
        )
        loss_fn = nn.MSELoss()  # Use MSE for stability
        
        for epoch in range(self.epochs):
            train_loss = self._train_epoch(self.model, dataloader, optimizer, loss_fn)
            
            # Check for training failure
            if np.isnan(train_loss) or np.isinf(train_loss):
                logger.error(f"Final training failed due to NaN/Inf loss at epoch {epoch+1}")
                break
                
            if self.verbose > 0:
                print(f"Final training - Epoch {epoch+1}/{self.epochs}: Train Loss = {train_loss:.4f}")
        
        return self.model

    def estimate(self, X_complex):
        """
        Make predictions on new data.
        
        Parameters:
            X_complex: Complex radar data
            
        Returns:
            Predictions
        """
        
        if self.model is None:
            raise ValueError("Model not trained. Call train_final_model first.")
        
        # Process complex data to normalized amplitude
        X_processed = self._process_complex_data(X_complex)
        
        # Create dataset and dataloader for prediction
        dataset = RadarData2Image(X_processed, np.zeros(len(X_processed)), self.processor)
        dataloader = DataLoader(dataset, batch_size=self.batch_size)
        
        self.model.eval()
        predictions = []
        
        with torch.no_grad():
            for batch in dataloader:
                inputs, _ = batch
                pixel_values = inputs['pixel_values'].to(self.device)
                outputs = self.model(pixel_values)
                predictions.extend(outputs.cpu().numpy())
        
        return np.array(predictions)

    def save_model(self, model_path):
        """
        Save the trained model.
        
        Parameters:
            model_path: Path to save the model
        """
        if self.model is None:
            logger.error("No model to save. Train the model first.")
            return
        
        torch.save(self.model.state_dict(), model_path)
        if self.verbose > 0:
            logger.info(f"Model saved to {model_path}")

    def load_model(self, model_path):
        """
        Load a pre-trained model.
        
        Parameters:
            model_path: Path to the saved model
        """
        if self.model is None:
            self.model = self.build_model()
        
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()
        if self.verbose > 0:
            logger.info(f"Model loaded from {model_path}")
        
        return self.model
