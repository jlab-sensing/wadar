# Important parameters that I want to stay consistent across different modules.

RANDOM_SEED = 42
KFOLD_SPLITS = 5

def bulk_density_to_label(bulk_density, soil_type = "silty"):
    """
    Convert bulk density to a label. For silty soil based on 
    https://www.nrcs.usda.gov/sites/default/files/2022-11/Bulk%20Density%20-%20Soil%20Health%20Guide_0.pdf.
    Labels are based on root growth potential.
    """

    if soil_type == "silty":
        if bulk_density < 1.4:
            return "Ideal"
        elif bulk_density > 1.75:
            return "Restricted"
        else:
            return "Non-ideal"

num2label = bulk_density_to_label       # So that if I want to change the function later for a different labeling scheme, I can do it in one place.