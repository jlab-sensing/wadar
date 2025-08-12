"""Standardized parameters throughout the pipeline."""

KFOLD_SPLITS = 5
RANDOM_SEED = 42
FEATURE_COUNT = 8
GRID_SEARCH_SCORING = "neg_mean_squared_error"

def bulk_density_to_label(bulk_density: float, soil_type: str = "silty") -> str:
    """
    Convert bulk density to a label. For silty soil based on 
    https://www.nrcs.usda.gov/sites/default/files/2022-11/Bulk%20Density%20-%20Soil%20Health%20Guide_0.pdf.
    Labels are based on root growth potential.
    """

    if soil_type == "silty":
        if bulk_density < 1.4:
            return "Ideal"
        else:
            return "Non-ideal"

num2label = bulk_density_to_label       # So that if I want to change the function later for a different labeling scheme, I can do it in one place.
