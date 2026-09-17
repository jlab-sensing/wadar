import numpy as np
from pathlib import Path

# Load the original Pie Ranch feature array and its corresponding
# bulk-density target labels.
# Each row in X should correspond to the label at the same index in y.
X = np.load("data/pie-ranch-dataset/X.npy")
y = np.load("data/pie-ranch-dataset/y.npy")

# Identify samples whose bulk-density label is NOT approximately 0.49 g/cm³.
# `np.isclose(y, 0.49, atol=0.01)` is True for labels within approximately
# ±0.01 of 0.49 (about 0.48–0.50). The `~` operator reverses this, so:
#   True  -> keep the sample
#   False -> remove the sample
keep_mask = ~np.isclose(y, 0.49, atol=0.01)

# Apply the same Boolean mask to both features and labels. This removes
# the selected near-0.49 g/cm³ samples while preserving alignment between
# each feature row and its corresponding bulk-density label.
X_filtered = X[keep_mask]
y_filtered = y[keep_mask]

# Create the output directory if it does not already exist.
out_dir = Path("data/pie-ranch-filtered")
out_dir.mkdir(parents=True, exist_ok=True)

# Save the filtered feature array and filtered target-label array as new
# NumPy files, leaving the original Pie Ranch dataset unchanged.
np.save(out_dir / "X.npy", X_filtered)
np.save(out_dir / "y.npy", y_filtered)

# Print a concise before/after summary:
# - number of samples
# - unique bulk-density labels, rounded to two decimal places
print(f"Original: n={len(y)}  labels={sorted(set(y.round(2)))}")
print(f"Filtered: n={len(y_filtered)}  labels={sorted(set(y_filtered.round(2)))}")
print(f"Saved to {out_dir}/")
