from matplotlib import pyplot as plt
import numpy as np
import pathlib
import cv2
import pandas as pd

def imagesc(image_path):

    file = pathlib.Path(image_path)                 # Convert the image path to a pathlib object
                                                    # to check if it exists.
    if not file.exists():          
        raise FileNotFoundError(f"File {image_path} does not exist.")
    
    image = cv2.imread(image_path)                  # Read the image using OpenCV
    cv2.imwrite("image.png", image)                 # Writing the image because I'm having issues with Qt

def preprocess_image(image_path, img_height=227, img_width=227):
    image = cv2.imread(image_path)                
    image = cv2.resize(image, (img_width, img_height))  # Resize the image to fit the model's needs
    return image


TEST_HARNESS = True
if TEST_HARNESS:
    df = pd.read_csv("data/compact-2-binary/dataset.csv")
    image_path = df.iloc[0]["filename"]                             # Get the first image path

    imagesc(image_path)