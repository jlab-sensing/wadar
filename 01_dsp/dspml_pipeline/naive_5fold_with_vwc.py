import sys
import time
import warnings

import numpy as np
import pandas as pd

from pathlib import Path
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import KFold
from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.impute import SimpleImputer


# ---------------------------------------------------------------------
# Project imports
# ---------------------------------------------------------------------
sys.path.insert(0, ".")
warnings.filterwarnings("ignore")

from dspml_pipeline.data.frame_loader import (
    process_frames,
    novelda_digital_downconvert,
)
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import (
    full_monty_features,
    process_feature_table,
)
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import (
    AutoencoderLearnedFeatures,
)
from dspml_pipeline.feature_estimation.ridge_regression import RidgeRegression
from dspml_pipeline.feature_estimation.random_forest import RandomForest
from dspml_pipeline.feature_estimation.xgboost_tree import XGBoostTree
from dspml_pipeline.feature_estimation.svr import SVRRegression
from dspml_pipeline.feature_estimation.mlp import MLPRegression


# ---------------------------------------------------------------------
# Experiment configuration
# ---------------------------------------------------------------------
# Bulk-density threshold used to convert regression predictions into
# a low-compaction / high-compaction classification decision.
TAU = 1.4

# Dataset paths. The moisture-condition ID is retained for traceability
# in the console output, but is not used as a model input.
DATASETS = [
    ("data/wet-0-soil-compaction-dataset", 0),
    ("data/wet-1-soil-compaction-dataset", 1),
    ("data/wet-2-soil-compaction-dataset", 2),
]

# Naive scan-level intradomain cross-validation.
# Note: scans from the same physical sample may appear in both training
# and testing folds if the sample contains multiple .frames captures.
kf = KFold(n_splits=5, shuffle=True, random_state=42)


# ---------------------------------------------------------------------
# Load, clean, and digitally downconvert radar scans
# ---------------------------------------------------------------------
print("=" * 92)
print("STEP 1: Loading radar scans and target metadata")
print("=" * 92)

X_all = []
y_all = []
vwc_all = []

for dataset_dir, moisture_id in DATASETS:
    dataset_dir = Path(dataset_dir)

    print(f"\nProcessing dataset: {dataset_dir}")
    print(f"Moisture-condition ID: {moisture_id}")

    # Load sample metadata containing bulk density and volumetric water content.
    metadata = pd.read_csv(dataset_dir / "data-log.csv")

    # Keep only rows associated with valid sample identifiers.
    metadata = metadata[metadata["Sample #"].notna()].copy()
    metadata["Sample #"] = metadata["Sample #"].astype(str)

    # Identify sample folders, excluding hidden folders.
    sample_folders = sorted(
        [
            folder
            for folder in dataset_dir.iterdir()
            if folder.is_dir() and not folder.name.startswith(".")
        ],
        key=lambda path: path.name,
    )

    dataset_scan_count = 0

    for folder in sample_folders:
        capture_files = sorted(folder.glob("*.frames"))

        if not capture_files:
            print(f"  Skipping {folder.name}: no .frames files found.")
            continue

        # Match the folder name to its metadata row.
        sample_rows = metadata[metadata["Sample #"] == folder.name]

        if sample_rows.empty:
            print(f"  Skipping {folder.name}: no matching metadata row found.")
            continue

        # If the metadata log contains repeated rows for a sample, use means.
        bulk_density = sample_rows["Bulk Density (g/cm^3)"].astype(float).mean()
        vwc = sample_rows["VWC (%)"].astype(float).mean()

        print(
            f"  Sample {folder.name}: "
            f"{len(capture_files)} capture(s), "
            f"BD={bulk_density:.3f} g/cm^3, "
            f"VWC={vwc:.2f}%"
        )

        for capture_file in capture_files:
            frame_data, params = process_frames(folder, capture_file.name)

            if frame_data is None:
                print(f"    Skipping {capture_file.name}: frame loading returned None.")
                continue

            # Replace large values deviating from each row median by more than 50.
            row_median = np.median(frame_data, axis=1, keepdims=True)
            outlier_mask = np.abs(frame_data - row_median) > 50

            frame_data_clean = frame_data.copy()
            frame_data_clean[outlier_mask] = np.broadcast_to(
                row_median,
                frame_data.shape,
            )[outlier_mask]

            # Apply digital downconversion independently to every radar channel.
            ddc_data = np.zeros_like(frame_data_clean, dtype=np.complex64)

            for channel_index in range(frame_data_clean.shape[1]):
                ddc_data[:, channel_index] = novelda_digital_downconvert(
                    frame_data_clean[:, channel_index]
                )

            X_all.append(ddc_data)
            y_all.append(bulk_density)
            vwc_all.append(vwc)

            dataset_scan_count += 1

    print(f"Completed {dataset_dir.name}: {dataset_scan_count} valid scan(s) loaded.")


# Convert lists of scans, labels, and VWC values into NumPy arrays.
if not X_all:
    raise RuntimeError(
        "No valid radar scans were loaded. Check dataset paths, sample folder "
        "names, data-log.csv sample identifiers, and .frames files."
    )

X = np.stack(X_all)
y = np.asarray(y_all, dtype=float)
vwc = np.asarray(vwc_all, dtype=float)

print("\nDataset summary")
print(f"  Total radar scans: {len(y)}")
print(f"  Radar scan array shape: {X.shape}")
print(f"  Bulk-density range: {y.min():.3f} to {y.max():.3f} g/cm^3")
print(f"  VWC range: {vwc.min():.2f}% to {vwc.max():.2f}%")
print(f"  Bulk-density classification threshold: {TAU:.2f} g/cm^3")


# ---------------------------------------------------------------------
# Optional moisture-only reference model
# ---------------------------------------------------------------------
# This reference model predicts bulk density from VWC only. It is printed
# separately and is not used to calculate any model-grid result column.
print("\n" + "=" * 92)
print("STEP 2: Evaluating VWC-only reference model")
print("=" * 92)

vwc_only_predictions = np.zeros(len(y), dtype=float)

for fold_index, (train_idx, test_idx) in enumerate(kf.split(X), start=1):
    vwc_model = LinearRegression()

    vwc_model.fit(
        vwc[train_idx].reshape(-1, 1),
        y[train_idx],
    )

    vwc_only_predictions[test_idx] = vwc_model.predict(
        vwc[test_idx].reshape(-1, 1)
    )

    print(
        f"  Fold {fold_index}/5: "
        f"train={len(train_idx)} scans, test={len(test_idx)} scans"
    )

vwc_only_rmse = np.sqrt(mean_squared_error(y, vwc_only_predictions))
vwc_only_mae = mean_absolute_error(y, vwc_only_predictions)
vwc_only_acc = np.mean(
    (vwc_only_predictions >= TAU) == (y >= TAU)
)

print("\nVWC-only reference results")
print(f"  RMSE: {vwc_only_rmse:.4f} g/cm^3")
print(f"  MAE: {vwc_only_mae:.4f} g/cm^3")
print(f"  Classification accuracy: {vwc_only_acc:.3f}")


# ---------------------------------------------------------------------
# Feature-extraction helper functions
# ---------------------------------------------------------------------
def handcrafted_features(X_train, X_test):
    """
    Extract handcrafted radar features and impute missing values.

    The mean imputer is fit only on the training fold to avoid leakage
    from test-fold data into the feature preprocessing stage.
    """
    train_features, _, _ = process_feature_table(
        full_monty_features(X_train, np.zeros(len(X_train)))
    )

    test_features, _, _ = process_feature_table(
        full_monty_features(X_test, np.zeros(len(X_test)))
    )

    train_features = np.asarray(train_features, dtype=float)
    test_features = np.asarray(test_features, dtype=float)

    imputer = SimpleImputer(strategy="mean")

    train_features = imputer.fit_transform(train_features)
    test_features = imputer.transform(test_features)

    return (
        np.asarray(train_features, dtype=float),
        np.asarray(test_features, dtype=float),
    )


def pca_like_features(feature_class, X_train, X_test, y_train=None):
    """
    Extract eight learned components using PCA or kernel PCA.

    The kPCA project class requires y_train in its constructor, whereas
    the standard PCA project class does not.
    """
    if y_train is None:
        feature_model = feature_class(X_train, n_components=8)
    else:
        feature_model = feature_class(
            X_train,
            y_train,
            n_components=8,
        )

    _, _, train_features = feature_model.full_monty()
    _, _, test_features = feature_model.transform(X_test)

    return (
        np.asarray(train_features, dtype=float),
        np.asarray(test_features, dtype=float),
    )


def autoencoder_features(X_train, X_test, y_train):
    """
    Train an autoencoder on flattened radar magnitude data and return
    learned features for both the training and held-out test folds.
    """
    def magnitude_matrix(data):
        return np.abs(data).reshape(len(data), -1).astype(float)

    train_magnitude = magnitude_matrix(X_train)
    test_magnitude = magnitude_matrix(X_test)

    autoencoder = AutoencoderLearnedFeatures(
        train_magnitude,
        y_train,
        epochs=50,
        batch_size=16,
    )

    train_features = np.asarray(
        autoencoder.full_monty(train_magnitude),
        dtype=float,
    )

    test_features = np.asarray(
        autoencoder.transform(test_magnitude),
        dtype=float,
    )

    return train_features, test_features


def add_vwc_feature(
    radar_features_train,
    radar_features_test,
    vwc_train,
    vwc_test,
):
    """
    Append raw VWC as one additional input feature to the radar feature vector.
    """
    train_features = np.hstack(
        [
            radar_features_train,
            vwc_train.reshape(-1, 1),
        ]
    )

    test_features = np.hstack(
        [
            radar_features_test,
            vwc_test.reshape(-1, 1),
        ]
    )

    return train_features, test_features


# ---------------------------------------------------------------------
# Define model and feature-extraction configurations
# ---------------------------------------------------------------------
MODELS = {
    "Ridge": lambda: RidgeRegression(),
    "RF": lambda: RandomForest(tune_model_params=False),
    "GBT": lambda: XGBoostTree(False),
    "SVR": lambda: SVRRegression(False),
    "MLP": lambda: MLPRegression(),
}

FEATURE_EXTRACTORS = {
    "Handcrafted": lambda X_train, X_test, y_train: handcrafted_features(
        X_train,
        X_test,
    ),
    "PCA": lambda X_train, X_test, y_train: pca_like_features(
        PCALearnedFeatures,
        X_train,
        X_test,
    ),
    "kPCA": lambda X_train, X_test, y_train: pca_like_features(
        kPCALearnedFeatures,
        X_train,
        X_test,
        y_train,
    ),
    "Autoencoder": lambda X_train, X_test, y_train: autoencoder_features(
        X_train,
        X_test,
        y_train,
    ),
}


# ---------------------------------------------------------------------
# Evaluate every model-feature-extraction combination
# ---------------------------------------------------------------------
print("\n" + "=" * 92)
print("STEP 3: Evaluating radar-feature + VWC model configurations")
print("=" * 92)
print("Cross-validation: 5-fold shuffled KFold, random_state=42")
print(
    "Training time includes feature extraction and estimator fitting. "
    "Inference time includes estimator prediction only."
)

print()
print(
    f"{'Model':<8}"
    f"{'FE':<13}"
    f"{'RMSE':>8}"
    f"{'MAE':>8}"
    f"{'Acc':>7}"
    f"{'Train ms':>11}"
    f"{'Infer ms':>11}"
)
print("-" * 72)

results = []

for model_name, create_model in MODELS.items():
    for feature_name, extract_features in FEATURE_EXTRACTORS.items():
        print(
            f"\nRunning configuration: "
            f"model={model_name}, feature extractor={feature_name}"
        )

        try:
            out_of_fold_predictions = np.zeros(len(y), dtype=float)

            total_training_time_ms = 0.0
            total_inference_time_ms = 0.0

            for fold_index, (train_idx, test_idx) in enumerate(
                kf.split(X),
                start=1,
            ):
                print(
                    f"  Fold {fold_index}/5: "
                    f"train={len(train_idx)}, test={len(test_idx)}"
                )

                # Time feature extraction and VWC concatenation.
                feature_start = time.perf_counter()

                radar_train_features, radar_test_features = extract_features(
                    X[train_idx],
                    X[test_idx],
                    y[train_idx],
                )

                # Ensure contiguous float64 feature arrays for downstream models.
                radar_train_features = np.ascontiguousarray(
                    radar_train_features,
                    dtype=np.float64,
                )

                radar_test_features = np.ascontiguousarray(
                    radar_test_features,
                    dtype=np.float64,
                )

                train_features, test_features = add_vwc_feature(
                    radar_train_features,
                    radar_test_features,
                    vwc[train_idx],
                    vwc[test_idx],
                )

                feature_time_ms = (
                    time.perf_counter() - feature_start
                ) * 1000.0

                # Train the selected regression estimator.
                estimator = create_model()

                training_start = time.perf_counter()

                estimator.full_monty(
                    train_features,
                    y[train_idx],
                )

                estimator_training_time_ms = (
                    time.perf_counter() - training_start
                ) * 1000.0

                # Estimate bulk density for held-out scans.
                inference_start = time.perf_counter()

                fold_predictions = estimator.estimate(test_features)

                inference_time_ms = (
                    time.perf_counter() - inference_start
                ) * 1000.0

                out_of_fold_predictions[test_idx] = fold_predictions

                # Total training time includes feature extraction and fitting.
                total_training_time_ms += (
                    feature_time_ms + estimator_training_time_ms
                )
                total_inference_time_ms += inference_time_ms

            # Regression metrics computed from all out-of-fold predictions.
            rmse = np.sqrt(
                mean_squared_error(y, out_of_fold_predictions)
            )

            mae = mean_absolute_error(
                y,
                out_of_fold_predictions,
            )

            # Convert regression outputs into threshold-based predictions.
            classification_accuracy = np.mean(
                (out_of_fold_predictions >= TAU) == (y >= TAU)
            )

            average_training_time_ms = (
                total_training_time_ms / kf.get_n_splits()
            )

            average_inference_time_ms = (
                total_inference_time_ms / kf.get_n_splits()
            )

            results.append(
                (
                    model_name,
                    feature_name,
                    rmse,
                    mae,
                    classification_accuracy,
                    average_training_time_ms,
                    average_inference_time_ms,
                )
            )

            print(
                f"{model_name:<8}"
                f"{feature_name:<13}"
                f"{rmse:8.4f}"
                f"{mae:8.4f}"
                f"{classification_accuracy:7.3f}"
                f"{average_training_time_ms:11.2f}"
                f"{average_inference_time_ms:11.2f}"
            )

        except Exception as error:
            print(
                f"{model_name:<8}"
                f"{feature_name:<13}"
                f"FAILED: {type(error).__name__}: {error}"
            )


# ---------------------------------------------------------------------
# Save results table
# ---------------------------------------------------------------------
results_df = pd.DataFrame(
    results,
    columns=[
        "Model",
        "FE",
        "RMSE",
        "MAE",
        "acc",
        "TrainMs",
        "InferMs",
    ],
)

output_file = "naive_intradomain_vwc_full_grid.csv"
results_df.to_csv(output_file, index=False)

print("\n" + "=" * 92)
print("STEP 4: Experiment complete")
print("=" * 92)
print(f"Saved model-grid results to: {output_file}")
print(f"Completed configurations: {len(results_df)}")
print("Saved columns: Model, FE, RMSE, MAE, acc, TrainMs, InferMs")
