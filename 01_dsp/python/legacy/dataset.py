# Functions to prepare the dataset

import pandas as pd 
import pathlib
import json
import sys
import numpy as np

def create_labels(data_dir):
    """
    Create the json labels for the dataset using the csv file.
    """

    data_dir = pathlib.Path(data_dir)
    print(f"Creating labels in {data_dir}")
    if not data_dir.exists():
        print(f"Error: The directory {data_dir} does not exist.")
        return
    df = pd.read_csv(data_dir / "data-log.csv")
    
    for index, row in df.iterrows():
        sample_dir = data_dir / row["Sample #"]
        bulk_density = row["Bulk Density (g/cm^3)"]
        if sample_dir.exists():
            json_file = sample_dir / "data_params.json"
            if not json_file.exists():
                with open(json_file, 'w') as f:
                    json.dump({"bulk-density": [bulk_density]}, f)
            else:
                with open(json_file, 'r') as f:
                    data = f.read()
                parsed = json.loads(data)
                parsed["bulk-density"].append(bulk_density)
                with open(json_file, 'w') as f:
                    json.dump(parsed, f)
        else:
            print(f"Warning: {sample_dir} does not exist. Skipping.")

def delete_labels(data_dir):
    """
    Purging labels from the dataset for relabeling.
    """

    data_dir = pathlib.Path(data_dir)
    if not data_dir.exists():
        print(f"Error: The directory {data_dir} does not exist.")
        return
    df = pd.read_csv(data_dir / "data-log.csv")
    
    for index, row in df.iterrows():
        sample_dir = data_dir / row["Sample #"]
        json_file = sample_dir / "data_params.json"
        if json_file.exists():
            json_file.unlink()

def create_dataset(data_dir):

    data_dir = pathlib.Path(data_dir)
    df = pd.DataFrame(columns=["filename", "label"])            # This is so that each file has it's "label" which
                                                                # is the bulk density at the moment. We can add more 
                                                                # as needed for the regression pipeline.                                 
    
    for item in data_dir.iterdir(): 
        if item.is_dir():                                           
            json_file = item / "data_params.json"               # Check if the JSON file exists. Apparently / 
                                                                # is an overloaded operator in pathlib. 
            if json_file.exists():
                with open(json_file, 'r') as f: 
                    data = f.read()

                parsed = json.loads(data)
                label = parsed.get("bulk-density")   
                label = np.median(label) if isinstance(label, list) else label
                
                for img_file in item.glob("*.png"):
                    data_path = data_dir / item.name / img_file.name
                    df = df._append({"filename": data_path, "label": label}, ignore_index=True)
            else:
                print(f"Warning: {json_file} does not exist in {item.name}. Skipping.")         # Just in case I forgot to add the JSON file.

    
    # Save the DataFrame to a CSV file
    df.to_csv(data_dir / "dataset.csv", index=False)

def main(argc, arc):
    args = sys.argv[1:]

    if len(args) == 1:
        data_dir = args[0]
    else:
        print("Usage: python dataset.py <data_dir>")
        return

    delete_labels(data_dir)
    create_labels(data_dir)
    create_dataset(data_dir)
    df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")
    print(df)

if __name__ == '__main__':
    main(sys.argv, len(sys.argv))