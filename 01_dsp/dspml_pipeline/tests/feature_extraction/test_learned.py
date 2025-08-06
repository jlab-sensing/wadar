import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
import numpy as np
from dspml_pipeline.plotting_tools.data_plotting import plot_feature_reduction
import matplotlib.pyplot as plt

from scipy import stats

def display_feature_importance(feature_array, feature_names, labels):
    correlations = []
    for i in range(feature_array.shape[1]):
        correlation, _ = stats.pointbiserialr(labels, feature_array[:, i])
        correlations.append((feature_names[i], correlation))

    correlations.sort(key=lambda x: abs(x[1]), reverse=True)

    for feature, corr in correlations:
        print(f"{feature}: {corr:.2f}")

if __name__ == "__main__":
    setup_logging(verbose=True)
    dataset_dirs = ["../../data/wet-0-soil-compaction-dataset",
                    "../../data/wet-1-soil-compaction-dataset",
                    "../../data/wet-2-soil-compaction-dataset"]
    target_dir = "../../data/training-dataset"
    frameLoader = FrameLoader(dataset_dirs, target_dir)
    # X, y = frameLoader.extract_data()
    # frameLoader.save_dataset()
    X, y = load_dataset(dataset_dir=target_dir)

    n_components = 2
    
    # pca_tool = PCALearnedFeatures(X, n_components=n_components)
    # pca_amp, pca_ang, pca_combined = pca_tool.full_monty()

    # plot_feature_reduction(y, pca_amp, "PCA of I/Q signal amplitude", show_plot=False)
    # plot_feature_reduction(y, pca_ang, "PCA of I/Q signal angle", show_plot=False)
    # plot_feature_reduction(y, pca_combined, "PCA of I/Q signal amplitude and angle", show_plot=False)
    # plt.show()

    # kpca_tool = kPCALearnedFeatures(X, y, n_components=n_components)
    # pca_amp, pca_ang, pca_combined = kpca_tool.full_monty()

    # plot_feature_reduction(y, pca_amp, "kPCA of I/Q signal amplitude", show_plot=False)
    # plot_feature_reduction(y, pca_ang, "kPCA of I/Q signal angle", show_plot=False)
    # plot_feature_reduction(y, pca_combined, "kPCA of I/Q signal amplitude and angle", show_plot=False)
    # plt.show()

    # import time
    # import numpy as np
    # import torch
    # import torch.nn.functional as F
    # from torch.utils.data import DataLoader
    # from sklearn.model_selection import train_test_split
    # from sklearn.preprocessing import MinMaxScaler

    # if torch.cuda.is_available():
    #     torch.backends.cudnn.deterministic = True

    # ##########################
    # ### SETTINGS
    # ##########################

    # # Device
    # device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    # print('Device:', device)

    # # Hyperparameters
    # random_seed = 123
    # learning_rate = 0.005
    # num_epochs = 100
    # batch_size = 256

    # # Architecture
    # num_features = 81920
    # num_hidden_1 = 32

    # X = np.abs(X) # TODO: Dont't leave it like this
    # X = X.reshape(X.shape[0], -1) # Flatten
    # X = MinMaxScaler().fit_transform(X)  # Better scaling
    
    # X_train, X_test, y_train, y_test = train_test_split(
    #     X, y, test_size=0.2, random_state=random_seed, stratify=y
    # )

    # # Convert numpy arrays to torch tensors
    # X_train_tensor = torch.tensor(X_train, dtype=torch.float32)
    # X_test_tensor = torch.tensor(X_test, dtype=torch.float32)
    # y_train_tensor = torch.tensor(y_train, dtype=torch.float32)
    # y_test_tensor = torch.tensor(y_test, dtype=torch.float32)

    # # Create TensorDatasets
    # train_dataset = torch.utils.data.TensorDataset(X_train_tensor, y_train_tensor)
    # test_dataset = torch.utils.data.TensorDataset(X_test_tensor, y_test_tensor)

    # # Create DataLoaders
    # train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True)
    # test_loader = DataLoader(dataset=test_dataset, batch_size=batch_size, shuffle=False)

    # ##########################
    # ### MODEL
    # ##########################

    # class Autoencoder(torch.nn.Module):

    #     def __init__(self, num_features):
    #         super(Autoencoder, self).__init__()
            
    #         ### ENCODER
            
    #         self.linear_1 = torch.nn.Linear(num_features, num_hidden_1)
    #         # The following to lones are not necessary, 
    #         # but used here to demonstrate how to access the weights
    #         # and use a different weight initialization.
    #         # By default, PyTorch uses Xavier/Glorot initialization, which
    #         # should usually be preferred.
    #         self.linear_1.weight.detach().normal_(0.0, 0.1)
    #         self.linear_1.bias.detach().zero_()
            
    #         ### DECODER
    #         self.linear_2 = torch.nn.Linear(num_hidden_1, num_features)
    #         self.linear_1.weight.detach().normal_(0.0, 0.1)
    #         self.linear_1.bias.detach().zero_()
            
    #     def encoder(self, x):
    #         encoded = self.linear_1(x)
    #         encoded = F.leaky_relu(encoded)
    #         return encoded
        
    #     def decoder(self, encoded_x):
    #         logits = self.linear_2(encoded_x)
    #         decoded = torch.sigmoid(logits)
    #         return decoded
            

    #     def forward(self, x):
            
    #         ### ENCODER
    #         encoded = self.encoder(x)
            
    #         ### DECODER
    #         decoded = self.decoder(encoded)
            
    #         return decoded

        
    # torch.manual_seed(random_seed)
    # model = Autoencoder(num_features=num_features)
    # model = model.to(device)

    # optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    # ## Training

    # start_time = time.time()
    # for epoch in range(num_epochs):
    #     for batch_idx, (features, targets) in enumerate(train_loader):
            
    #         # don't need labels, only the images (features)
    #         features = features.view(-1, num_features).to(device)
                
    #         ### FORWARD AND BACK PROP
    #         decoded = model(features)
    #         cost = F.mse_loss(decoded, features)
    #         optimizer.zero_grad()
            
    #         cost.backward()
            
    #         ### UPDATE MODEL PARAMETERS
    #         optimizer.step()
            
    #         ### LOGGING
    #         if not batch_idx % 50:
    #             print ('Epoch: %03d/%03d | Batch %03d/%03d | Cost: %.4f' 
    #                 %(epoch+1, num_epochs, batch_idx, 
    #                     len(train_loader), cost))
                
    #     print('Time elapsed: %.2f min' % ((time.time() - start_time)/60))
        
    # print('Total Training Time: %.2f min' % ((time.time() - start_time)/60))

    # # Get encoded features from the entire set
    # model.eval()
    # with torch.no_grad():
    #     encoded_all = model.encoder(torch.tensor(X, dtype=torch.float32).to(device)).cpu().numpy()

    X_amp = np.abs(X)
    autoencoder = AutoencoderLearnedFeatures(X_amp, y, epochs=10, batch_size=256, verbose=True)
    encoded_all = autoencoder.full_monty(X_amp)

    # Plot the encoded features
    plot_feature_reduction(y, encoded_all, "Autoencoder Feature Reduction (All Data)", show_plot=True)