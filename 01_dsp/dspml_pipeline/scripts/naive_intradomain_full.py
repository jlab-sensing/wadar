"""
File:
    naive_intradomain_full.py

Description:
    Run LOOCV on each individual run (including between compaction levels).

Authors:
    jLab
    Maansa Krovvidi
    Claude

Version:
    0.0.1
"""
import sys, time, warnings, numpy as np, pandas as pd, collections
from pathlib import Path
sys.path.insert(0, '.'); warnings.filterwarnings("ignore")

from dspml_pipeline.data.frame_loader import process_frames, novelda_digital_downconvert
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import full_monty_features, process_feature_table
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
from dspml_pipeline.feature_estimation.ridge_regression import RidgeRegression
from dspml_pipeline.feature_estimation.random_forest import RandomForest
from dspml_pipeline.feature_estimation.xgboost_tree import XGBoostTree
from dspml_pipeline.feature_estimation.svr import SVRRegression
from dspml_pipeline.feature_estimation.mlp import MLPRegression
from sklearn.model_selection import KFold
from sklearn.metrics import mean_absolute_error, mean_squared_error
from sklearn.impute import SimpleImputer


TAU, BAND = 1.4, 0.10
DATASETS = [
    ("data/wet-0-soil-compaction-dataset", 0),
    ("data/wet-1-soil-compaction-dataset", 1),
    ("data/wet-2-soil-compaction-dataset", 2),
]


def naive_intradomain_full():
    """
    """
    # ---- build lab data with AVERAGED (mean) tin labels ----
    X_all, y_all = [], []
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
                X_all.append(ddc)
                y_all.append(bulk_density)

    X = np.stack(X_all)
    y = np.array(y_all)
    print(f"scans={len(y)}  range={y.min():.2f}-{y.max():.2f}")

    base = np.sqrt(np.mean((y - y.mean())**2))
    majority = max(collections.Counter(y >= TAU).values()) / len(y)
    print(f"mean-predictor baseline RMSE = {base:.4f}   majority-class baseline = {majority:.3f}\n")

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

    kf = KFold(n_splits=5, shuffle=True, random_state=42)

    rows = []
    print(f"{'Model':<8}{'FE':<13}{'RMSE':>8}{'MAE':>8}{'acc':>7}{'near_acc':>9}{'Train ms':>10}{'Infer ms':>10}")
    print("-"*72)
    for mn, mk in MODELS.items():
        FE = {"Handcrafted": lambda A,B,ytr: handcrafted(A,B),
              "PCA":         lambda A,B,ytr: pca_like(PCALearnedFeatures, A, B),
              "kPCA":        lambda A,B,ytr: pca_like(kPCALearnedFeatures, A, B, ytr),
              "Autoencoder": lambda A,B,ytr: autoenc(A, B, ytr)}
        for fn, ff in FE.items():
            try:
                p_all = np.zeros(len(y))
                total_train_ms, total_infer_ms = 0.0, 0.0
                for tr, te in kf.split(X):
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
                near = np.abs(y - TAU) <= BAND
                near_acc = np.mean((p_all[near] >= TAU) == (y[near] >= TAU)) if near.sum() else float('nan')

                # average per-fold timing (5 folds)
                avg_train_ms = total_train_ms / 5
                avg_infer_ms = total_infer_ms / 5

                rows.append((mn, fn, rmse, mae, acc, near_acc, avg_train_ms, avg_infer_ms))
                print(f"{mn:<8}{fn:<13}{rmse:8.4f}{mae:8.4f}{acc:7.3f}{near_acc:9.3f}{avg_train_ms:10.2f}{avg_infer_ms:10.2f}")
            except Exception as e:
                print(f"{mn:<8}{fn:<13}  FAILED: {type(e).__name__}: {e}")
    df = pd.DataFrame(rows, columns=["Model","FE","RMSE","MAE","acc","near_acc","TrainMs","InferMs"])
    df.to_csv("naive_intradomain_full_grid.csv", index=False)
    print(f"\nsaved naive_intradomain_full_grid.csv")
    print(f"baseline RMSE = {base:.4f}  |  majority-class accuracy = {majority:.3f}")
    print(f"configs beating RMSE baseline: {(df.RMSE < base).sum()} of {len(df)}")

if __name__ == "__main__":
    naive_intradomain_full()
