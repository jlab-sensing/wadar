import numpy as np

from _03_hephaestus import feature_tools

# struggling to make a universal monte carlo feature selection function so here are some helpers

def monte_carlo_selected_items(feature_array):
    top_n = np.random.randint(1, feature_array.shape[1] + 1)
    selected_indices = np.random.choice(feature_array.shape[1], top_n, replace=False)
    selected_features = feature_array[:, selected_indices]
    return selected_indices,selected_features