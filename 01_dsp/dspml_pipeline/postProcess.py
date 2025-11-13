"""
postProcess.py

Convert CSV outputs into better-ingestible formats.

Author:
    jLab

Date:
    12 Nov 2025

Version:
    0.0.9
"""
import argparse
import csv
import os

from typing import Any, List


def _read_csv(path: str) -> List[dict]:
    """
    """
    rows = []
    with open(path, "r") as csvp:
        reader = csv.DictReader(csvp)
        for row in reader:
            rows.append(row)
    return rows

def process_input_file(path: str):
    """
    """
    # Check if input file is valid format and exists.
    if (os.path.isfile(path) and path.split(".")[-1] == "csv"):
        print(f"Great success!: {path}")
    else:
        print(f"Bummer!: {path}")

def combine_results(paths: List[str]):
    # Group results.
    results = {}
    for path in paths:
        data = _read_csv(path)
        for row in data:
            try:
                label = "-".join([row["Feature"], row["Model"]])
            except KeyError as e:
                print(f"WARNING! - Skipping {path}: {e}")
                continue

            if label not in results.keys():
                results[label] = {
                    "R2": [],
                    "RMSE": [],
                    "Accuracy": []
                }

            try:
                results[label]["R2"].append(row["R2"])
                results[label]["RMSE"].append(row["RMSE"])
                results[label]["Accuracy"].append(row["Accuracy"])
            except KeyError:
                print(f"WARNING! - Skipping {path} since these results are bad!: {e}")
                continue

    print(results)

def process_input_dir(path: str):
    """
    """
    # Check if input directory exists.
    if (os.path.isdir(path)):
        print(f"Great success!: {path}")
    else:
        print(f"Bummer!: {path}")
        return

    # Gather all CSV files from directory.
    contents = os.listdir(path)
    csv_paths = [
        os.path.join(path, f) for f in contents if f.split(".")[-1] == "csv"
    ]

    # Collect combined results.
    results = combine_results([p for p in csv_paths if "results" in p])

def post_process(path_input_file: str, path_input_dir: str):
    """
    Ingest an input CSV file and output a filtered CSV.
    """
    if path_input_file:
        process_input_file(path_input_file)
    if path_input_dir:
        process_input_dir(path_input_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="postProcess",
        description="Output bulk info on or simplified data."
    )
    parser.add_argument(
        "--input_file",
        "-f",
        default="",
        help="Single file input."
    )
    parser.add_argument(
        "--input_dir",
        "-d",
        default="",
        help="Directory (batch) input."
    )
    args = parser.parse_args()
    post_process(path_input_file=args.input_file, path_input_dir=args.input_dir)
