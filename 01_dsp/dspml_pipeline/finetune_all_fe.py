import sys, copy, warnings, numpy as np, pandas as pd
sys.path.insert(0, '.'); warnings.filterwarnings("ignore")

import torch
from dspml_pipeline.data.frame_loader import load_dataset
from dspml_pipeline.feature_extraction.handcrafted.feature_tools import full_monty_features, process_feature_table
from dspml_pipeline.feature_extraction.learned.pca import PCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.kpca import kPCALearnedFeatures
from dspml_pipeline.feature_extraction.learned.autoencoder import AutoencoderLearnedFeatures
from dspml_pipeline.feature_estimation.mlp import MLPRegression
from sklearn.metrics import mean_squared_error
from sklearn.impute import SimpleImputer

TAU = 1.4
N_CALIB = 3
N_DRAWS = 30
FT_EPOCHS = 20
FT_LR = 1e-4
rng = np.random.default_rng(42)

# ---- EDIT THIS: pick the source/target split ----
Xl, yl = load_dataset("data/lab-dataset")
X1, y1 = load_dataset("data/field1-cache")
X2, y2 = load_dataset("data/field2-cache")
Xtr = np.concatenate([Xl, X1, X2])
ytr = np.concatenate([yl, y1, y2])
Xte, yte = load_dataset("data/pie-ranch-dataset")
SPLIT_NAME = "Lab+Field-1+Field-2 -> Pie Ranch"
# ---------------------------------------------------

print(f"SPLIT: {SPLIT_NAME}")
print(f"train n={len(ytr)}  test n={len(yte)}\n")

def amp(Z):
    return np.abs(Z).reshape(len(Z), -1).astype(float)

def handcrafted_fe(Xtr, Xte):
    a,_,_ = process_feature_table(full_monty_features(Xtr, np.zeros(len(Xtr))))
    b,_,_ = process_feature_table(full_monty_features(Xte, np.zeros(len(Xte))))
    a = np.asarray(a, float); b = np.asarray(b, float)
    im = SimpleImputer(strategy="mean").fit(a)
    return np.asarray(im.transform(a), float), np.asarray(im.transform(b), float)

def pca_fe(cls, Xtr, Xte, y_train=None):
    m = cls(Xtr, y_train, n_components=8) if y_train is not None else cls(Xtr, n_components=8)
    _, _, Ftr = m.full_monty()
    _, _, Fte = m.transform(Xte)
    return np.asarray(Ftr, float), np.asarray(Fte, float)

def autoenc_fe(Xtr, Xte, y_train):
    Atr, Ate = amp(Xtr), amp(Xte)
    ae = AutoencoderLearnedFeatures(Atr, y_train, epochs=50, batch_size=16)
    Ftr = np.asarray(ae.full_monty(Atr), float)
    Fte = np.asarray(ae.transform(Ate), float)
    return Ftr, Fte

FE = {
    "Handcrafted": lambda: handcrafted_fe(Xtr, Xte),
    "PCA":         lambda: pca_fe(PCALearnedFeatures, Xtr, Xte),
    "kPCA":        lambda: pca_fe(kPCALearnedFeatures, Xtr, Xte, ytr),
    "Autoencoder": lambda: autoenc_fe(Xtr, Xte, ytr),
}

def fine_tune(model_obj, F_calib, y_calib, epochs=FT_EPOCHS, lr=FT_LR):
    model = model_obj.model
    scaler = model_obj.scaler
    Xc = scaler.transform(F_calib)
    Xc_t = torch.tensor(Xc, dtype=torch.float32)
    yc_t = torch.tensor(y_calib, dtype=torch.float32).reshape(-1, 1)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = torch.nn.MSELoss()
    model.train()
    for _ in range(epochs):
        optimizer.zero_grad()
        pred = model(Xc_t)
        loss = loss_fn(pred, yc_t)
        loss.backward()
        optimizer.step()
    model.eval()
    return model_obj

oracle_rmse = np.sqrt(np.mean((yte - yte.mean())**2))
n = len(yte)
n_calib = min(N_CALIB, n - 1)
idx = np.arange(n)

all_results = []
print(f"{'FE':<13}{'no_FT_RMSE':>12}{'no_FT_xbase':>12}{'FT_RMSE_mean':>14}{'FT_RMSE_std':>13}{'FT_xbase':>10}")
print("-"*80)

for fe_name, fe_fn in FE.items():
    try:
        Ftr, Fte_all = fe_fn()
        Ftr = np.ascontiguousarray(Ftr, dtype=np.float64)
        Fte_all = np.ascontiguousarray(Fte_all, dtype=np.float64)

        mlp = MLPRegression()
        mlp.full_monty(Ftr, ytr)
        baseline_state = copy.deepcopy(mlp.model.state_dict())

        yp_no_ft = mlp.estimate(Fte_all)
        no_ft_rmse = np.sqrt(mean_squared_error(yte, yp_no_ft))

        ft_rmses = []
        for draw in range(N_DRAWS):
            calib_idx = rng.choice(idx, size=n_calib, replace=False)
            eval_idx = np.setdiff1d(idx, calib_idx)
            F_calib, y_calib = Fte_all[calib_idx], yte[calib_idx]
            F_eval, y_eval = Fte_all[eval_idx], yte[eval_idx]

            mlp.model.load_state_dict(copy.deepcopy(baseline_state))
            fine_tune(mlp, F_calib, y_calib)
            yp_ft = mlp.estimate(F_eval)
            ft_rmses.append(np.sqrt(mean_squared_error(y_eval, yp_ft)))

        ft_rmses = np.array(ft_rmses)
        print(f"{fe_name:<13}{no_ft_rmse:12.4f}{no_ft_rmse/oracle_rmse:12.2f}"
              f"{ft_rmses.mean():14.4f}{ft_rmses.std():13.4f}{ft_rmses.mean()/oracle_rmse:10.2f}")

        for draw, r in enumerate(ft_rmses):
            all_results.append(dict(FE=fe_name, draw=draw, ft_rmse=r,
                                     no_ft_rmse=no_ft_rmse, oracle_rmse=oracle_rmse))
    except Exception as e:
        print(f"{fe_name:<13}  FAILED: {type(e).__name__}: {e}")

df = pd.DataFrame(all_results)
df.to_csv(f"finetune_all_fe_{SPLIT_NAME.replace(' ','').replace('+','').replace('>','to')}.csv", index=False)
print(f"\nsaved results  |  oracle baseline = {oracle_rmse:.4f}")
