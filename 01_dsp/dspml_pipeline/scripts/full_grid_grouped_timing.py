"""
File:
    full_grid_grouped_timing.py

Description:
    LOOCV for all experimental scnenarios.

Authors:
    jLab
    Maansa Krovvidi
    Claude

Date:
    16 Sep 2026

Version:
    0.0.1
"""
import sys, time, warnings, numpy as np, pandas as pd
sys.path.insert(0, '.'); warnings.filterwarnings("ignore")

from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
from dspml_pipeline.feature_estimation.ridge_regression import RidgeRegression
from dspml_pipeline.feature_estimation.random_forest import RandomForest
from dspml_pipeline.feature_estimation.xgboost_tree import XGBoostTree
from dspml_pipeline.feature_estimation.svr import SVRRegression
from dspml_pipeline.feature_estimation.mlp import MLPRegression
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import full_monty_features, process_feature_table
from sklearn.model_selection import LeaveOneGroupOut
from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.impute import SimpleImputer
import pandas as pd

TAU = 1.4

def load(label_mode):
    """label_mode: 'avg' = mean of all tins per run, 'top' = first tin only"""
    from pathlib import Path
    from dspml_pipeline.data.frame_loader import process_frames, novelda_digital_downconvert

    DATASETS = [
        ("data/wet-0-soil-compaction-dataset", 0),
        ("data/wet-1-soil-compaction-dataset", 1),
        ("data/wet-2-soil-compaction-dataset", 2),
    ]

    X_list, y_list, run_group, moist_group = [], [], [], []
    run_id = 0
    for dataset_dir, moist_id in DATASETS:
        dataset_dir = Path(dataset_dir)
        df = pd.read_csv(dataset_dir / "data-log.csv")
        df = df[df["Sample #"].notna()]
        df["Sample #"] = df["Sample #"].astype(str)
        subdirs = sorted([d for d in dataset_dir.iterdir() if d.is_dir() and not d.name.startswith(".")], key=lambda p: p.name)
        for folder in subdirs:
            capture_files = sorted(folder.glob("*.frames"))
            if not capture_files:
                continue
            sample_rows = df[df["Sample #"] == folder.name]
            if sample_rows.empty:
                continue
            bulk_density = sample_rows["Bulk Density (g/cm^3)"].astype(float).mean()
            for capture_file in capture_files:
                frame_data, params = process_frames(folder, capture_file.name)
                if frame_data is None:
                    continue
                median = np.median(frame_data, axis=1, keepdims=True)
                mask = np.abs(frame_data - median) > 50
                frame_data_clean = frame_data.copy()
                frame_data_clean[mask] = np.broadcast_to(median, frame_data.shape)[mask]
                ddc = np.zeros_like(frame_data_clean, dtype=np.complex64)
                for i in range(frame_data_clean.shape[1]):
                    ddc[:, i] = novelda_digital_downconvert(frame_data_clean[:, i])
                X_list.append(ddc); y_list.append(bulk_density)
                run_group.append(run_id); moist_group.append(moist_id)
            run_id += 1  # correctly indented: once per folder/run

    return np.stack(X_list), np.array(y_list), np.array(run_group), np.array(moist_group)

def handcrafted(Xtr, Xte):
    a,_,_ = process_feature_table(full_monty_features(Xtr, np.zeros(len(Xtr))))
    b,_,_ = process_feature_table(full_monty_features(Xte, np.zeros(len(Xte))))
    a = np.asarray(a, float); b = np.asarray(b, float)
    im = SimpleImputer(strategy="mean").fit(a)
    return np.asarray(im.transform(a), float), np.asarray(im.transform(b), float)

def pca_like(cls, Xtr, Xte, y_train=None):
    m = cls(Xtr, y_train, n_components=8) if y_train is not None else cls(Xtr, n_components=8)
    _, _, Ftr = m.full_monty()
    _, _, Fte = m.transform(Xte)
    return np.asarray(Ftr, float), np.asarray(Fte, float)

def autoenc(Xtr, Xte, y_train):
    amp = lambda Z: np.abs(Z).reshape(len(Z), -1).astype(float)
    Atr, Ate = amp(Xtr), amp(Xte)
    ae = AutoencoderLearnedFeatures(Atr, y_train, epochs=50, batch_size=16)
    Ftr = np.asarray(ae.full_monty(Atr), float)
    Fte = np.asarray(ae.transform(Ate), float)
    return Ftr, Fte

MODELS = {"Ridge": lambda: RidgeRegression(),
          "RF":    lambda: RandomForest(tune_model_params=False),
          "GBT":   lambda: XGBoostTree(False),
          "SVR":   lambda: SVRRegression(False),
          "MLP":   lambda: MLPRegression()}

def run_all(X, y, groups, group_name, label_mode):
    logo = LeaveOneGroupOut()
    n_folds = len(set(groups))
    print(f"\n=== {group_name} | labels={label_mode} | N={n_folds} folds ===")
    print(f"{'Model':<8}{'FE':<13}{'RMSE':>8}{'MAE':>8}{'acc':>7}{'Train_ms':>10}{'Infer_ms':>10}")
    print("-"*64)
    rows = []
    for mn, mk in MODELS.items():
        FE = {"Handcrafted": lambda Xtr,Xte,ytr: handcrafted(Xtr,Xte),
              "PCA":         lambda Xtr,Xte,ytr: pca_like(PCALearnedFeatures, Xtr, Xte),
              "kPCA":        lambda Xtr,Xte,ytr: pca_like(kPCALearnedFeatures, Xtr, Xte, ytr),
              "Autoencoder": lambda Xtr,Xte,ytr: autoenc(Xtr, Xte, ytr)}
        for fn, ff in FE.items():
            try:
                p_all = np.zeros(len(y))
                total_train_ms, total_infer_ms = 0.0, 0.0
                for tr, te in logo.split(X, y, groups):
                    t0 = time.perf_counter()
                    Ftr, Fte = ff(X[tr], X[te], y[tr])
                    Ftr = np.ascontiguousarray(Ftr, dtype=np.float64)
                    Fte = np.ascontiguousarray(Fte, dtype=np.float64)
                    feat_time = (time.perf_counter() - t0) * 1000

                    est = mk()
                    t0 = time.perf_counter()
                    est.full_monty(Ftr, y[tr])
                    total_train_ms += (time.perf_counter() - t0) * 1000 + feat_time

                    t0 = time.perf_counter()
                    p_fold = est.estimate(Fte)
                    total_infer_ms += (time.perf_counter() - t0) * 1000

                    p_all[te] = p_fold

                rmse = np.sqrt(mean_squared_error(y, p_all))
                mae = mean_absolute_error(y, p_all)
                acc = np.mean((p_all >= TAU) == (y >= TAU))
                avg_train_ms = total_train_ms / n_folds
                avg_infer_ms = total_infer_ms / n_folds

                rows.append((mn, fn, rmse, mae, acc, avg_train_ms, avg_infer_ms))
                print(f"{mn:<8}{fn:<13}{rmse:8.4f}{mae:8.4f}{acc:7.3f}{avg_train_ms:10.2f}{avg_infer_ms:10.2f}")
            except Exception as e:
                print(f"{mn:<8}{fn:<13}  FAILED: {type(e).__name__}: {e}")
    return pd.DataFrame(rows, columns=["Model","FE","RMSE","MAE","acc","TrainMs","InferMs"])

def full_grid_grouped_timing():
    all_results = []
    for label_mode in ["avg", "top"]:
        X, y, run_group, moist_group = load(label_mode)
        print(f"\n########## LABEL MODE: {label_mode}  (n={len(y)}, range {y.min():.2f}-{y.max():.2f}) ##########")
        df1 = run_all(X, y, run_group, "Leave-one-run-out (13 groups)", label_mode)
        df1["Grouping"] = "run"; df1["Labels"] = label_mode
        df2 = run_all(X, y, moist_group, "Leave-one-moisture-out (3 groups)", label_mode)
        df2["Grouping"] = "moisture"; df2["Labels"] = label_mode
        all_results += [df1, df2]

    pd.concat(all_results).to_csv("full_grid_grouped_results_timing.csv", index=False)
    print("\nsaved full_grid_grouped_results_timing.csv")

if __name__ == "__main__":
    full_grid_grouped_timing()
