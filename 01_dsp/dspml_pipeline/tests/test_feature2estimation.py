import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np

from dspml_pipeline.data.frame_loader import FrameLoader, load_dataset
from dspml_pipeline.setup_logging import setup_logging
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import full_monty_features, save_feature_table, load_feature_table, process_feature_table
from dspml_pipeline.feature_extraction.handcrafted.feature_pruning import lasso_minimize_features, correlation_minimize_features, mutual_info_minimize_features
from dspml_pipeline.feature_estimation.eval_tools import evaluate_classic_models, evaluate_deep_models, validate_classical_models, classical_models_full_monty
from dspml_pipeline.results import load_results, display_feature_results
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
from dspml_pipeline.feature_extraction.learned.cnn import CNNLearnedFeatures
from dspml_pipeline.plotting_tools.data_plotting import plot_feature_reduction

from scipy import stats
import matplotlib.pyplot as plt

HANDCRAFTED = False
PCA = False
KPCA = False
AUTOENCODER = False
CNN = True

TUNE_MODEL_PARAMS = False # Because tuning with a grid search is time laborious

if __name__ == "__main__":

    setup_logging(verbose=True)

    dataset_dirs = ["../data/wet-0-soil-compaction-dataset",
                    "../data/wet-1-soil-compaction-dataset",
                    "../data/wet-2-soil-compaction-dataset",
                    "../data/field-soil-compaction-dataset"]
    target_dir = "../data/training-dataset"
    frameLoader = FrameLoader(dataset_dirs, target_dir)
    # X_train, y_train = frameLoader.extract_data()
    # frameLoader.save_dataset()
    X_train, y_train = load_dataset(dataset_dir=target_dir)

    validation_dirs = ["../data/field-2-soil-compaction-dataset"]
    validation_target_dir = "../data/validation-dataset"
    frameLoader = FrameLoader(validation_dirs, validation_target_dir)
    # X_val, y_val = frameLoader.extract_data()
    # frameLoader.save_dataset()
    X_val, y_val = load_dataset(dataset_dir=validation_target_dir)
     
    if HANDCRAFTED:

        tune_model_params = False # Because tuning with a grid search is time laborious
        
        # feature_table = full_monty_features(X=X, label=y)
        # save_feature_table(feature_table, target_dir)
        # training_feature_table = full_monty_features(X=X_train, label=y_train)
        # save_feature_table(training_feature_table, target_dir)
        training_feature_table, training_feature_array, training_feature_names, training_labels = load_feature_table(directory=target_dir)

        # training_corr_feature_table, training_corr_features = correlation_minimize_features(feature_table=training_feature_table)
        # training_corr_feature_array, training_corr_feature_names, training_corr_labels = process_feature_table(training_corr_feature_table)

        # validation_feature_table = full_monty_features(X=X_val, label=y_val)
        # save_feature_table(validation_feature_table, validation_target_dir)
        validation_feature_table, validation_feature_array, validation_feature_names, validation_labels = load_feature_table(directory=validation_target_dir)

        feature_name = "Handcrafted"
        

        ridgeRegressor, randomForest, gbTree, svr = evaluate_classic_models(target_dir, training_feature_array, training_labels, tune_model_params, feature_name)
        evaluate_deep_models(target_dir, validation_feature_array, validation_labels, feature_name)
        results_df = load_results(target_dir)
        # display_feature_results(feature_name, results_df)

        # Predict on the validation dataset using all trained models
        models = {
            "Ridge Regression": ridgeRegressor,
            "Random Forest": randomForest,
            "Gradient Boosting": gbTree,
            "SVR": svr
        }
        validate_classical_models(validation_target_dir, validation_feature_array, validation_labels, feature_name, models)
        results_df = load_results(validation_target_dir)
        display_feature_results(feature_name, results_df)

    if PCA:

        n_components = 8
        tune_model_params = False # Because tuning with a grid search is time laborious

        pca_train_tool = PCALearnedFeatures(X_train, n_components=n_components)
        pca_train_amplitude, pca_train_phase, pca_train_combined = pca_train_tool.full_monty()
        pca_val_amplitude, pca_val_phase, pca_val_combined = pca_train_tool.transform(X_val)

        # Amplitude
        feature_name_amp = "PCA Amplitude"

        # classical_models_full_monty(target_dir, y_train, validation_target_dir, y_val, tune_model_params, pca_train_amplitude, pca_val_amplitude, feature_name_amp)
        classical_models_full_monty(
            training_dir = target_dir,
            training_labels = y_train,
            validation_dir = validation_target_dir,
            validation_labels = y_val,
            tune_model_params = tune_model_params,
            training_features = pca_train_amplitude,
            validation_features = pca_val_amplitude,
            feature_name = feature_name_amp
        )

        # Phase
        feature_name_ang = "PCA Phase"

        classical_models_full_monty(
            training_dir = target_dir,
            training_labels = y_train,
            validation_dir = validation_target_dir,
            validation_labels = y_val,
            tune_model_params = tune_model_params,
            training_features = pca_train_phase,
            validation_features = pca_val_phase,
            feature_name = feature_name_ang
        )
            
        # Combined
        feature_name_combined = "PCA Combined"

        classical_models_full_monty(
            training_dir = target_dir,
            training_labels = y_train,
            validation_dir = validation_target_dir,
            validation_labels = y_val,
            tune_model_params = tune_model_params,
            training_features = pca_train_combined,
            validation_features = pca_val_combined,
            feature_name = feature_name_combined
        )

    if KPCA:

        n_components = 8
        tune_model_params = False # Because tuning with a grid search is time laborious

        kpca_train_tool = kPCALearnedFeatures(X_train, y_train, n_components=n_components)
        kpca_train_amplitude, kpca_train_phase, kpca_train_combined = kpca_train_tool.full_monty()
        kpca_val_amplitude, kpca_val_phase, kpca_val_combined = kpca_train_tool.transform(X_val)

        # Amplitude
        feature_name_amp = "kPCA Amplitude"
        classical_models_full_monty(
            training_dir = target_dir,
            training_labels = y_train,
            validation_dir = validation_target_dir,
            validation_labels = y_val,
            tune_model_params = tune_model_params,
            training_features = kpca_train_amplitude,
            validation_features = kpca_val_amplitude,
            feature_name = feature_name_amp
        )

        # Phase
        feature_name_ang = "kPCA Phase"
        classical_models_full_monty(
            training_dir = target_dir,
            training_labels = y_train,
            validation_dir = validation_target_dir,
            validation_labels = y_val,
            tune_model_params = TUNE_MODEL_PARAMS,
            training_features = kpca_train_phase,
            validation_features = kpca_val_phase,
            feature_name = feature_name_ang
        )

        # Combined
        feature_name_combined = "kPCA Combined"
        classical_models_full_monty(
            training_dir = target_dir,
            training_labels = y_train,
            validation_dir = validation_target_dir,
            validation_labels = y_val,
            tune_model_params = tune_model_params,
            training_features = kpca_train_combined,
            validation_features = kpca_val_combined,
            feature_name = feature_name_combined
        )

    if AUTOENCODER:
        epochs = 1000

        # Amplitude
        X_train_amp = np.abs(X_train)
        X_val_amp = np.abs(X_val)
        autoencoder_amp = AutoencoderLearnedFeatures(X_train_amp, y_train, epochs=epochs, batch_size=256, verbose=False)
        encoded_train_amp = autoencoder_amp.full_monty(X_train_amp)
        encoded_val_amp = autoencoder_amp.transform(X_val_amp)
        feature_name_amp = "Autoencoder Amplitude"
        classical_models_full_monty(
            training_dir=target_dir,
            training_labels=y_train,
            validation_dir=validation_target_dir,
            validation_labels=y_val,
            tune_model_params=TUNE_MODEL_PARAMS,
            training_features=encoded_train_amp,
            validation_features=encoded_val_amp,
            feature_name=feature_name_amp
        )

        # Phase
        X_train_pha = np.unwrap(np.angle(X_train))
        X_val_pha = np.unwrap(np.angle(X_val))
        autoencoder_pha = AutoencoderLearnedFeatures(X_train_pha, y_train, epochs=epochs, batch_size=256, verbose=False)
        encoded_train_pha = autoencoder_pha.full_monty(X_train_pha)
        encoded_val_pha = autoencoder_pha.transform(X_val_pha)
        feature_name_pha = "Autoencoder Phase"
        classical_models_full_monty(
            training_dir=target_dir,
            training_labels=y_train,
            validation_dir=validation_target_dir,
            validation_labels=y_val,
            tune_model_params=TUNE_MODEL_PARAMS,
            training_features=encoded_train_pha,
            validation_features=encoded_val_pha,
            feature_name=feature_name_pha
        )

        # Combined
        X_train_com = np.concatenate((X_train_amp, X_train_pha), axis=1)
        X_val_com = np.concatenate((X_val_amp, X_val_pha), axis=1)
        autoencoder_com = AutoencoderLearnedFeatures(X_train_com, y_train, epochs=epochs, batch_size=256, verbose=False)
        encoded_train_com = autoencoder_com.full_monty(X_train_com)
        encoded_val_com = autoencoder_com.transform(X_val_com)
        feature_name_com = "Autoencoder Combined"
        classical_models_full_monty(
            training_dir=target_dir,
            training_labels=y_train,
            validation_dir=validation_target_dir,
            validation_labels=y_val,
            tune_model_params=TUNE_MODEL_PARAMS,
            training_features=encoded_train_com,
            validation_features=encoded_val_com,
            feature_name=feature_name_com
        )

    if CNN:
        epochs = 20

        # Amplitude
        X_train_amp = np.abs(X_train)
        X_val_amp = np.abs(X_val)
        cnn_amp = CNNLearnedFeatures(X_train_amp, y_train, epochs=epochs, verbose=False)
        features_train_amp = cnn_amp.full_monty(X_train_amp)
        features_val_amp = cnn_amp.transform(X_val_amp)
        feature_name_amp = "CNN Amplitude"
        classical_models_full_monty(
            training_dir=target_dir,
            training_labels=y_train,
            validation_dir=validation_target_dir,
            validation_labels=y_val,
            tune_model_params=TUNE_MODEL_PARAMS,
            training_features=features_train_amp,
            validation_features=features_val_amp,
            feature_name=feature_name_amp
        )

        # Phase
        X_train_pha = np.unwrap(np.angle(X_train))
        X_val_pha = np.unwrap(np.angle(X_val))
        cnn_pha = CNNLearnedFeatures(X_train_pha, y_train, epochs=epochs, verbose=False)
        features_train_pha = cnn_pha.full_monty(X_train_pha)
        features_val_pha = cnn_pha.transform(X_val_pha)
        feature_name_pha = "CNN Phase"
        classical_models_full_monty(
            training_dir=target_dir,
            training_labels=y_train,
            validation_dir=validation_target_dir,
            validation_labels=y_val,
            tune_model_params=TUNE_MODEL_PARAMS,
            training_features=features_train_pha,
            validation_features=features_val_pha,
            feature_name=feature_name_pha
        )

        # Combined
        X_train_com = np.concatenate((X_train_amp, X_train_pha), axis=1)
        X_val_com = np.concatenate((X_val_amp, X_val_pha), axis=1)
        cnn_com = CNNLearnedFeatures(X_train_com, y_train, epochs=epochs, verbose=False)
        features_train_com = cnn_com.full_monty(X_train_com)
        features_val_com = cnn_com.transform(X_val_com)
        feature_name_com = "CNN Combined"
        classical_models_full_monty(
            training_dir=target_dir,
            training_labels=y_train,
            validation_dir=validation_target_dir,
            validation_labels=y_val,
            tune_model_params=TUNE_MODEL_PARAMS,
            training_features=features_train_com,
            validation_features=features_val_com,
            feature_name=feature_name_com
        )