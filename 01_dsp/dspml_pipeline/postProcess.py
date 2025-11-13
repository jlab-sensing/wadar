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

import numpy as np

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

def _write_csv(data: List[str], path: str):
    """
    """
    with open(path, "w") as csvp:
        writer = csv.writer(csvp, delimiter=",")
        for row in data:
            writer.writerow(row)

def process_input_file(path: str):
    """
    """
    # Check if input file is valid format and exists.
    if (os.path.isfile(path) and path.split(".")[-1] == "csv"):
        print(f"Great success!: {path}")
    else:
        print(f"Bummer!: {path}")

def ingest_results_paths(paths: List[str]) -> dict:
    """
    Convert CSV file paths into formatted data.
    """
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
                    "R2": { "Raw": [], "Mean": 0.0, "STD": 0.0 },
                    "RMSE": { "Raw": [], "Mean": 0.0, "STD": 0.0 },
                    "Accuracy": { "Raw": [], "Mean": 0.0, "STD": 0.0 },
                    "N": 0
                }

            try:
                results[label]["R2"]["Raw"].append(float(row["R2"]))
                results[label]["RMSE"]["Raw"].append(float(row["RMSE"]))
                results[label]["Accuracy"]["Raw"].append(float(row["Accuracy"]))
                results[label]["N"] += 1
            except KeyError:
                print(f"WARNING! - Skipping {path} since these results are bad!: {e}")
                continue
    return results

def process_results(results: dict):
    """
    """
    for label in results.keys():
        for metric in ["R2", "RMSE", "Accuracy"]:
            try:
                results[label][metric]["Mean"] = np.mean(
                    np.array(results[label][metric]["Raw"])
                )
                results[label][metric]["STD"] = np.std(
                    np.array(results[label][metric]["Raw"])
                )
            except Exception as e:
                print(f"ERROR: We got {e} from {results[label][metric]['Raw']}")

def combine_results(paths: List[str]):
    """
    """
    results = ingest_results_paths(paths)
    process_results(results)
    return results

def save_results(results: dict, base_path: str):
    """
    """
    data = []
    # First add a header. 
    data.append([
        "Feature-Model",
        "R2 (mean)",
        "R2 (std)",
        "RMSE (mean)",
        "RMSE (std)",
        "N"
    ])

    # Next format results into a list of CSV rows.
    for key in results.keys():
        data.append([
            key,
            results[key]["R2"]["Mean"],
            results[key]["R2"]["STD"],
            results[key]["RMSE"]["Mean"],
            results[key]["RMSE"]["STD"],
            results[key]["N"]
        ])

    # Combined data path.
    combined_path = os.path.join(base_path, "results_val-combined.csv")
    _write_csv(data, combined_path)

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
    results = combine_results([p for p in csv_paths if "val_results" in p])

    # Present or save results.
    save_results(results=results, base_path=path)
    """
    [print(f"{label} - {metric} = {results[label][metric]['Mean']}") for metric in [
        #"R2", "RMSE", "Accuracy"
            "R2", "RMSE"
        ] for label in results.keys()
    ]
    """


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
