import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)) # https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _05_apollo import viz_tools
from scipy.signal import welch, find_peaks, hilbert
from scipy.fft import fft, fftfreq
from scipy.optimize import curve_fit
from scipy.linalg import lstsq
import pandas as pd
import seaborn as sns

def main():
    
    # Load the dataset
    dataset_dir = "../data/training-dataset"
    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y
    

if __name__ == "__main__":
    main()
