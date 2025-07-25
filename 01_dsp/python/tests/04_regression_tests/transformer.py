# transformer based regression

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _06_hermes.logger import update_results

import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
import requests
from PIL import Image
from transformers import BlipProcessor, BlipForConditionalGeneration
from sklearn.model_selection import train_test_split

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

        Returns:
            int: Number of samples in the dataset.
        """
        return len(self.X)

    def __getitem__(self, idx):
        """
        Gets an item from the dataset.

        Args:
        """
        img = Image.fromarray((self.X[idx] * 255).astype(np.uint8)).convert("RGB")      # Converts radar data at idx into an 8-bit RGB image.
        inputs = self.processor(images=img, return_tensors="pt")
        inputs = {k: v.squeeze(0) for k, v in inputs.items()}                           # Turns image into tensor and removes the batch dimension.
        label = torch.tensor(self.y[idx], dtype=torch.float32)                          # Prepares the label as a PyTorch float tensor.
        return inputs, label
    
class TransformerRegressionHead(nn.Module):
    def __init__(self, encoder, output_dim=1024):
        super().__init__()
        self.encoder = encoder.vision_model             # Only using to vision encoder because the other parts of the model are for captioning.
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.regressor = nn.Linear(output_dim, 1)       # Encoder's output is averaged (pooled) and run through a linear layer to produce a single regression output.

    def forward(self, pixel_values):
        outputs = self.encoder(pixel_values=pixel_values)
        last_hidden_state = outputs.last_hidden_state                       # [batch, patch_count, hidden_dim]
        pooled = self.pool(last_hidden_state.transpose(1, 2)).squeeze(-1)   # [B, hidden_dim] -> [B, 1]
        return self.regressor(pooled).squeeze(-1)                           # [B, 1] -> [B]

def train(model, dataloader, optimizer, loss_fn, device):
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

if __name__ == "__main__":

    # Set device to GPU if available, otherwise CPU
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # Loads model from https://huggingface.co/Salesforce/blip-image-captioning-large
    processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-large")
    blip = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-large").to(device)

    # Load and split the data
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X = np.abs(hydros.X)
    y = hydros.y
    X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2)

    # Wraps the radar data into a PyTorch Dataset
    train_dataset = RadarData2Image(X_train, y_train, processor)
    val_dataset = RadarData2Image(X_val, y_val, processor)
    train_loader = DataLoader(train_dataset, batch_size=1, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=1)

    # Loading data into the model in batches. 
    model = TransformerRegressionHead(blip, output_dim=1024).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    loss_fn = nn.L1Loss()

    # Standard training loop
    for epoch in range(10):
        train_loss = train(model, train_loader, optimizer, loss_fn, device)
        val_loss, preds, trues = evaluate(model, val_loader, loss_fn, device)
        print(f"Epoch {epoch}: Train Loss = {train_loss:.4f}, Val Loss = {val_loss:.4f}")


# if __name__ == "__main__":

#     dataset_dir = "../../data/combined-soil-compaction-dataset"
#     hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
#     X = np.abs(hydros.X)
#     y = hydros.y

#     # https://huggingface.co/Salesforce/blip-image-captioning-large


#     processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-large")
#     model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-large").to("cuda")

#     # freeze the model parameters
#     for param in model.parameters():
#         param.requires_grad = False


        
#     regression_model = RegressionHead(model).to("cuda")
#     regression_model

#     # img_url = 'https://storage.googleapis.com/sfr-vision-language-research/BLIP/demo.jpg' 
#     # raw_image = Image.open(requests.get(img_url, stream=True).raw).convert('RGB')

#     # # conditional image captioning
#     # text = "a photography of"
#     # inputs = processor(raw_image, text, return_tensors="pt").to("cuda")

#     # out = model.generate(**inputs)
#     # print(processor.decode(out[0], skip_special_tokens=True))

#     # # unconditional image captioning
#     # inputs = processor(raw_image, return_tensors="pt").to("cuda")

#     # out = model.generate(**inputs)
#     # print(processor.decode(out[0], skip_special_tokens=True))
