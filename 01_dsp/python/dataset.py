# Functions to prepare the dataset

import pandas as pd 
import pathlib
import json

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
                
                for img_file in item.glob("*.png"):
                    data_path = data_dir / item.name / img_file.name
                    df = df._append({"filename": data_path, "label": label}, ignore_index=True)
            else:
                print(f"Warning: {json_file} does not exist in {item.name}. Skipping.")         # Just in case I forgot to add the JSON file.

    
    # Save the DataFrame to a CSV file
    df.to_csv(data_dir / "dataset.csv", index=False)

# ==========
# Test harness
# ==========
TEST_HARNESS = True
if TEST_HARNESS:
    data_dir = "data/compact-3"
    create_dataset(data_dir)
    df = pd.read_csv(pathlib.Path(data_dir) / "dataset.csv")
    print(df)