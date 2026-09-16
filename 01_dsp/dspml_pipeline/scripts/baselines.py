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
    14 Sep 2026

Version:
    0.0.1
"""
import argparse
import logging
import os
import pandas as pd

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
    # Data entered into this dict are in format of (idxW, idxC), where:
    #   idxW == "wetness index" and
    #   idxC == "compaction index"
    groups = {}

    # Gather data into groups.
    logger.info("Hi nub.")
    

def baseline(path_input: str):
    """
    baseline(path_input)

    Generate baseline RMSE calculations for comparison to GOPHERS models.
    """
    # Load all relevant data.
    df = load_data(path=path_input)
    print(df)

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
