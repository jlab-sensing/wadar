"""
File:
    view_data

Description:
    View the contents of a saved numpy file.

Author:
    jLab
    nubby
    Perplexity.AI

Date:
    24 Feb 2026

Version:
    1.0.0
"""
import argparse
import numpy as np
import os

from typing import Union


def _load_npy_file(path: str) -> Union[np.array, None]:
    """
    _load_npy_file(path)
    
    Load the contents of a saved .npy file if proper format;
    otherwise return None.

    Args:
        path    (str)   Path to file.

    Returns:
        data    (np.array, None)
    """
    try:
        assert(os.path.isfile(path) and path.split(".")[-1] == "npy")
        data = np.load(path)
    except AssertionError:
        data = None

    return data


def view_data(path: str):
    # Load the file in question.
    data = _load_npy_file(path=path)

    try:
        print(f"Contents: {data}")
        print(f"Shape: {data.shape}")
    except AttributeError:
        print(f"ERROR: File {path} invalid; check the path!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="View the contents of an input .npy file.")
    parser.add_argument(
            "--path",
            "-p",
            required=True,
            type=str,
            help="Path to the desired file."
        )
    args = parser.parse_args()
    view_data(path=args.path)
