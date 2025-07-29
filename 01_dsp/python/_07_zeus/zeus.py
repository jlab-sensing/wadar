# Zeus: End-to-End Evaluation Script

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'  
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='tensorflow')
import sys
import yaml

# Add parent directory to path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
sys.path.insert(0, parent_dir)

# Core imports
from _01_gaia.loader import FrameLoader
from _01_gaia.dataset import combine_datasets

# Zeus modules
from evaluators import evaluate_all_models
from feature_processor import process_handcrafted_features, process_pca_features, process_autoencoder_features
from visualization import generate_results_summary, plot_reduced_features, tsne_plot
import numpy as np
import matplotlib.pyplot as plt

def main():
    """
    Main Zeus pipeline execution.
    """

    # Load configuration
    print("[INFO] Loading Zeus configuration...")
    with open("zeus_params.yaml", "r") as f:
        zeus_params = yaml.safe_load(f)

    # Extract configuration
    raw_training_datasets = zeus_params['data']['training']['raw_datasets']
    target_training_datasets = zeus_params['data']['training']['target_dataset']
    raw_validation_datasets = zeus_params['data']['validation']['raw_datasets']
    target_validation_dataset = zeus_params['data']['validation']['target_dataset']
    new_dataset = zeus_params['data']['new_dataset']

    print(f"[INFO] Training dataset: {target_training_datasets}")
    print(f"[INFO] Validation dataset: {target_validation_dataset}")
    print(f"[INFO] New dataset mode: {new_dataset}")

    # ====================================================
    # Data Loading and Preparation
    # ====================================================

    # Combine datasets if requested
    if new_dataset:
        print("[INFO] Combining raw datasets...")
        combine_datasets(raw_training_datasets, target_training_datasets, verbose=True)
        combine_datasets(raw_validation_datasets, target_validation_dataset, verbose=True)
    
    # Load the training and validation datasets
    print("[INFO] Loading training and validation datasets...")
    training_frame_loader = FrameLoader(target_training_datasets, new_dataset=new_dataset, ddc_flag=True, verbose=new_dataset)
    X_train, y_train = training_frame_loader.X, training_frame_loader.y
    
    validation_frame_loader = FrameLoader(target_validation_dataset, new_dataset=new_dataset, ddc_flag=True, verbose=new_dataset)
    X_val, y_val = validation_frame_loader.X, validation_frame_loader.y

    print(f"[INFO] Training data shape: {X_train.shape}")
    print(f"[INFO] Validation data shape: {X_val.shape}")

    # ====================================================
    # Handcrafted Features
    # ====================================================

    if zeus_params['features']['handcrafted']['enabled']:
        print("\n" + "="*60)
        print("HANDCRAFTED FEATURES EVALUATION")
        print("="*60)
        
        training_features, validation_features, training_labels, validation_labels = process_handcrafted_features(
            X_train, y_train, X_val, y_val, target_training_datasets, target_validation_dataset, 
            zeus_params, new_dataset
        )

        evaluate_all_models(
            zeus_params=zeus_params,
            training_dataset=target_training_datasets,
            validation_dataset=target_validation_dataset,
            training_labels=training_labels,
            validation_labels=validation_labels,
            training_features=training_features,
            validation_features=validation_features,
            feature_type_name="Handcrafted Features"
        )
        
        print("[INFO] Handcrafted features evaluation completed.")

    # ====================================================
    # PCA-based Features
    # ====================================================

    if zeus_params['features']['pca']['enabled']:
        print("\n" + "="*60)
        print("PCA-BASED FEATURES EVALUATION")
        print("="*60)
        
        pca_features = process_pca_features(X_train, y_train, X_val, y_val, zeus_params)

        # Evaluate PCA Amplitude features
        print("\n[INFO] Evaluating PCA Amplitude features...")
        evaluate_all_models(
            zeus_params=zeus_params,
            training_dataset=target_training_datasets,
            validation_dataset=target_validation_dataset,
            training_labels=pca_features['amplitude']['labels_train'],
            validation_labels=pca_features['amplitude']['labels_val'],
            training_features=pca_features['amplitude']['train'],
            validation_features=pca_features['amplitude']['val'],
            feature_type_name="PCA Amplitude"
        )

        # Evaluate PCA Phase features
        print("\n[INFO] Evaluating PCA Phase features...")
        evaluate_all_models(
            zeus_params=zeus_params,
            training_dataset=target_training_datasets,
            validation_dataset=target_validation_dataset,
            training_labels=pca_features['phase']['labels_train'],
            validation_labels=pca_features['phase']['labels_val'],
            training_features=pca_features['phase']['train'],
            validation_features=pca_features['phase']['val'],
            feature_type_name="PCA Phase"
        )
        
        # Evaluate PCA Combined features
        print("\n[INFO] Evaluating PCA Combined features...")
        evaluate_all_models(
            zeus_params=zeus_params,
            training_dataset=target_training_datasets,
            validation_dataset=target_validation_dataset,
            training_labels=pca_features['combined']['labels_train'],
            validation_labels=pca_features['combined']['labels_val'],
            training_features=pca_features['combined']['train'],
            validation_features=pca_features['combined']['val'],
            feature_type_name="PCA Combined"
        )


        if zeus_params['features']['pca']['visualize']:

            print("[INFO] Visualizing PCA features...")
            
            reduced_training_features_amplitude = pca_features['amplitude']['train']
            reduced_validation_features_amplitude = pca_features['amplitude']['val']
            reduced_training_features_phase = pca_features['phase']['train']
            reduced_validation_features_phase = pca_features['phase']['val']
            reduced_training_features_combined = pca_features['combined']['train']
            reduced_validation_features_combined = pca_features['combined']['val']
            labels_train = pca_features['combined']['labels_train']
            labels_val = pca_features['combined']['labels_val']

            # tsne_plot(
            #     reduced_training_features_amplitude, reduced_validation_features_amplitude, labels_train, labels_val
            # )
            # tsne_plot(
            #     reduced_training_features_phase, reduced_validation_features_phase, labels_train, labels_val
            # )
            # tsne_plot(
            #     reduced_training_features_combined, reduced_validation_features_combined, labels_train, labels_val
            # )

            plot_reduced_features(
                reduced_training_features_amplitude, reduced_validation_features_amplitude, labels_train, labels_val, "PCA Amplitude"
            )
            plot_reduced_features(
                reduced_training_features_phase, reduced_validation_features_phase, labels_train, labels_val, "PCA Phase"
            )
            # plot_reduced_features( 
            #     reduced_training_features_combined, reduced_validation_features_combined, labels_train, labels_val
            # )
            plt.show()

        print("[INFO] PCA-based features evaluation completed.")    # ====================================================
    # Autoencoder-based Features 
    # ====================================================

    if zeus_params['features']['autoencoder']['enabled']:
        print("\n" + "="*60)
        print("AUTOENCODER-BASED FEATURES EVALUATION")
        print("="*60)

        autoencoder_features = process_autoencoder_features(X_train, y_train, X_val, y_val, zeus_params)

        # Evaluate Autoencoder Amplitude features
        print("\n[INFO] Evaluating Autoencoder Amplitude features...")
        evaluate_all_models(
            zeus_params=zeus_params,
            training_dataset=target_training_datasets,
            validation_dataset=target_validation_dataset,
            training_labels=autoencoder_features['amplitude']['labels_train'],
            validation_labels=autoencoder_features['amplitude']['labels_val'],
            training_features=autoencoder_features['amplitude']['train'],
            validation_features=autoencoder_features['amplitude']['val'],
            feature_type_name="Autoencoder Amplitude"
        )

        # Evaluate Autoencoder Phase features
        print("\n[INFO] Evaluating Autoencoder Phase features...")
        evaluate_all_models(
            zeus_params=zeus_params,
            training_dataset=target_training_datasets,
            validation_dataset=target_validation_dataset,
            training_labels=autoencoder_features['phase']['labels_train'],
            validation_labels=autoencoder_features['phase']['labels_val'],
            training_features=autoencoder_features['phase']['train'],
            validation_features=autoencoder_features['phase']['val'],
            feature_type_name="Autoencoder Phase"
        )
        
        # Evaluate Autoencoder Combined features
        print("\n[INFO] Evaluating Autoencoder Combined features...")
        evaluate_all_models(
            zeus_params=zeus_params,
            training_dataset=target_training_datasets,
            validation_dataset=target_validation_dataset,
            training_labels=autoencoder_features['combined']['labels_train'],
            validation_labels=autoencoder_features['combined']['labels_val'],
            training_features=autoencoder_features['combined']['train'],
            validation_features=autoencoder_features['combined']['val'],
            feature_type_name="Autoencoder Combined"
        )

        # Visualization
        if zeus_params['features']['autoencoder']['visualize']:
            print("\n[INFO] Visualizing autoencoder features...")
            
            # Visualize Autoencoder features
            plot_reduced_features(
                autoencoder_features['amplitude']['train'], 
                autoencoder_features['amplitude']['val'], 
                autoencoder_features['amplitude']['labels_train'], 
                autoencoder_features['amplitude']['labels_val'], 
                "Autoencoder Amplitude Features"
            )
            
            plot_reduced_features(
                autoencoder_features['phase']['train'], 
                autoencoder_features['phase']['val'], 
                autoencoder_features['phase']['labels_train'], 
                autoencoder_features['phase']['labels_val'], 
                "Autoencoder Phase Features"
            )
            
            plot_reduced_features(
                autoencoder_features['combined']['train'], 
                autoencoder_features['combined']['val'], 
                autoencoder_features['combined']['labels_train'], 
                autoencoder_features['combined']['labels_val'], 
                "Autoencoder Combined Features"
            )
            
            plt.show()

        print("[INFO] Autoencoder-based features evaluation completed.")
        
    # ====================================================
    # Results Summary
    # ====================================================

    # print("\n" + "="*60)
    # print("RESULTS SUMMARY")
    # print("="*60)
    
    # generate_results_summary(target_validation_dataset, top_n=10)
    
    print("\n[INFO] Zeus evaluation pipeline completed successfully!")



if __name__ == "__main__":
    main()