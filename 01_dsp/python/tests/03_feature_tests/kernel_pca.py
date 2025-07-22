import logging
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir))# https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir
sys.path.insert(0, parent_dir)

from _01_gaia.loader import FrameLoader
from _05_apollo import viz_tools
from _04_athena import tree
from _03_hephaestus import feature_tools
from _05_apollo import viz_tools

from sklearn.decomposition import KernelPCA
from mpl_toolkits.mplot3d import Axes3D
from sklearn.preprocessing import StandardScaler

if __name__ == "__main__":
    
    dataset_dir = "../../data/combined-soil-compaction-dataset"
    feature_file_name = "features.csv"
    test_size = 0.2
    encoding_dim = 3

    hydros = FrameLoader(dataset_dir, new_dataset=False, ddc_flag=True)
    X, y = hydros.X, hydros.y 

    # Kernel PCA Feature Selection

    X = abs(X)

    N, R, T = X.shape
    X_flat = X.reshape(N, R * T)

    # s = StandardScaler()
    # X_flat = s.fit_transform(X_flat)

    # Using a random forest model, we can create a Monte Carlo simulation to evaluate the performance of different Kernel PCA parameters.

    best_mae = float('inf')
    best_params = None

    param_grid = {
        'kernel': ['poly', 'rbf', 'sigmoid'],
        'gamma': np.logspace(-3, 3, 100).tolist(), 
        'degree': np.linspace(1, 3, 3).tolist()  # degree > 3 seems to blow up since we have numbers in the 4000s and 5000s
    }


    for i in range(50):

        kernel = np.random.choice(param_grid['kernel'])
        gamma = np.random.choice(param_grid['gamma'])
        degree = np.random.choice(param_grid['degree']) # ignored when not poly anyway

        try:
            transformer = KernelPCA(n_components=encoding_dim, kernel=kernel, gamma=gamma, degree=degree)
            encoded_features = transformer.fit_transform(X_flat)
        except Exception as e:
            continue



        # Train Gradient Boosted Tree model
        model, metrics = tree.train_gradient_boosted_tree(
            encoded_features,
            y,
            n_estimators=100
        )

        mae = metrics['mae']

        if mae < best_mae:
            best_mae = mae
            best_params = (kernel, gamma, degree)

        # print every 10 iterations
        if (i + 1) % 10 == 0:
            print(f"Iteration {i+1} | Current best performing parameters: {best_params} | Current best MAE: {best_mae}")
        

        
    print("Best Model Determined:   ", best_params)
    print("Best MAE:                ", best_mae)

    # best params plotted
    kernel, gamma, degree = best_params
    transformer = KernelPCA(n_components=encoding_dim, kernel=kernel, gamma=gamma, degree=degree)
    encoded_features = transformer.fit_transform(X_flat)

    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(
        encoded_features[:, 0], 
        encoded_features[:, 1], 
        encoded_features[:, 2], 
        c=y, cmap='viridis', alpha=0.5
    )
    ax.set_xlabel('PCA Component 1')
    ax.set_ylabel('PCA Component 2')
    ax.set_zlabel('PCA Component 3')
    fig.colorbar(sc, label='Label')
    plt.show()

    # Right now, I'm benchmarking the autoencoder against a random forest model
    # trained on the original features. In the future, this should be replaced
    # with the best perfmorming model(s). 

    # # Train Random Forest model on encoded features
    # model, metrics_autoencoder = tree.train_random_forest(
    #     encoded_features, y, test_size=test_size, n_estimators=100)

    # # Train Random Forest model on manually selected features
    # _, feature_array, feature_names, labels = feature_tools.load_feature_table(
    #     dataset_dir, "feature_random_forest_monte_carlo.csv")
    
    # model_rf, metrics_manual = tree.train_random_forest(
    #     feature_array,
    #     labels,
    #     test_size=test_size,
    #     n_estimators=100
    # )

    # print("Manually selected features metrics:", metrics_manual)
    # print("Autoencoder selected features metrics:", metrics_autoencoder)