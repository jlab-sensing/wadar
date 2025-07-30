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
from _03_hephaestus import feature_tools

# TensorFlow imports

from sklearn.linear_model import RidgeClassifier, Ridge

from adapt.utils import make_classification_da

from adapt.feature_based import CORAL
from sklearn.decomposition import PCA

from xgboost import XGBRegressor

import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, Model, Input
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.svm import SVR
from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.pipeline import make_pipeline

def main():

    # Load your features (same as before)
    training_data = "../data/training-dataset"
    validation_data = "../data/validation-dataset"

    Xs_table, Xs, _, ys = feature_tools.load_feature_table(training_data, "features.csv")
    Xt_table, Xt, _, yt = feature_tools.load_feature_table(validation_data, "features.csv")

    def evaluate(model_name, model):
        coral = CORAL(estimator=model)
        coral.fit(Xs, ys.flatten(), Xt=Xt)

        preds = coral.predict(Xt)
        mse = mean_squared_error(yt, preds)
        mae = mean_absolute_error(yt, preds)

        print(f"[CORAL + {model_name}] Target MSE: {mse:.6f}, MAE: {mae:.6f}")

    # Try different base regressors
    evaluate("Ridge", Ridge(alpha=1.0))
    evaluate("SVR", SVR(kernel="rbf"))
    evaluate("XGBoost", XGBRegressor(n_estimators=100, max_depth=5, random_state=42))
    evaluate("Ridge (scaled)", make_pipeline(StandardScaler(), Ridge(alpha=1.0)))


if __name__ == "__main__":
    main()