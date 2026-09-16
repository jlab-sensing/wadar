"""
Name:
    baseline.py

Description:
    Output baseline model results.

Author:
    nubby
    Maansa Krovvidi
    David Glover

Date:
    15 Sep 2026

Version:
    0.0.2
"""
import argparse
import logging
import numpy as np
import os
import pandas as pd


# Configs.
logger = logging.getLogger("baseline")


def load_data(path: str) -> pd.DataFrame:
    """
    load_data()

    Load and pre-format data containing labels for GOPHERS evals.
    """
    # TODO: Ingest data from native format (i.e. from the same format as rest
    #       of dspml_pipeline).
    # NOTE: Currently can only load labels from custom-generated bulk label
    #       sheet. This sheet was downloaded as part of the sheet found here:
    #
    #       https://docs.google.com/spreadsheets/d/
    #           1ZyaHpjoSAfjLH66a4lvrTM9T-h2HPTiXToLWXfZYSSM/edit?usp=sharing
    if not os.path.isfile(path):
        logger.error(f"File {path} does not exist!")
    if not path.endswith(".csv"):
        logger.error("File {path} improper format!")

    logger.info(f"Loading {path}...")
    return pd.read_csv(path)

def do_loocv(df: pd.DataFrame):
    """
    do_loocv(df)
    """
    # NOTE: This currently only works with custom exported list.
    #       See "load_data()" for further clarification.
    # TODO: Extract formatting of "groups" to another function to allow for
    #       more modular inclusion of other input data formats.
    groups = {}

    # First get identifiers for each scenario.
    for scene in df["Tin Label"].unique():
        # Set labels as the average of all GT measurements.
        groups[scene] = {
                "sbd": np.mean(df.loc[
                    df["Tin Label"] == scene,
                    "Bulk Density (g/cm^3)"].values),
                "vwc": np.mean(df.loc[
                    df["Tin Label"] == scene,
                    "VWC (%)"].values)
            }

    # Iteratively work through each scene:
    se = 0.0                # Squared-error is easy to track incrementally.
    n = len(groups.keys())  # Number of unique labels.
    for scene in groups.keys():
        # Calculate fold residual by difference between held-out label and
        # average of all others.
        y_hat = np.mean([groups[s]["sbd"] for s in groups.keys() if s != scene])
        y = groups[scene]["sbd"]
        res = (y - y_hat)   # Residual for scene.
        se += res**2        # Square the error.
    mse = se / n            # Take the mean of the incrementally-squared error.
    rmse = np.sqrt(mse)     # And here is the RMSE.

    # Gather data into groups.
    logger.info(f"LOOCV (in lab):\t{rmse}")
    

def baseline(path_input: str):
    """
    baseline(path_input)

    Generate baseline RMSE calculations for comparison to GOPHERS models.
    """
    # Load all relevant data.
    df = load_data(path=path_input)

    # Report on inter-fold RMSE for mean predictor.
    do_loocv(df=df)

if __name__ == "__main__":
    # Logger setup.
    logging.basicConfig(level=logging.INFO)

    # Parse args for later triage.
    parser = argparse.ArgumentParser(
        description=(
            "Generate simple baselines for comparison to GOPHERS models."
        )
    )
    parser.add_argument(
            "--input-path",
            type=str,
            default="data/",
            help="Path to input directory containing GOPHERS inputs."
        )
    args = parser.parse_args()

    baseline(path_input=args.input_path)
