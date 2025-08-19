# dspml

## Running the pipeline

1. Clone and install:
```bash
git clone https://github.com/jlab-sensing/wadar.git
cd wadar/01_dsp/dspml_pipeline
python -m venv venv 
source venv/bin/activate
pip install --no-cache-dir -r requirements.txt
```

The venv can be reactivated at anytime using
```
python -m venv venv 
source venv/bin/activate
```


2. Run using a config file:
```bash
python scripts/main.py scripts/config.yaml
```

## Configuration

The pipeline is controlled through a YAML configuration file with the following sections:

### Data Configuration
```yaml
data:       
  label_name: "Bulk Density (g/cm^3)"       # Target variable in data_log.csv
  new_dataset: false                        # Set true for first time processing
  training:       
    dataset_dirs:                           # Raw training data directories
      - "../data/wet-0-soil-compaction-dataset"
      - "../data/field-soil-compaction-dataset"
    target_dir: "../data/training-dataset"  # Output directory for processed data
  validation:
    dataset_dirs:                           # Raw validation data directories
      - "../data/field-pie-ranch-dataset"
    target_dir: "../data/validation-dataset"
```

### Feature Configuration
```yaml
# Handcrafted feature settings
handcrafted:
  enabled: false                # Enable feature extraction
  new_features: true            # Regenerate features
  pruning_method: all           # Feature selection: corr, mi, lasso, none
  top_n: 16                     # Number of features to keep

# Learned feature settings
learned:
  n_features: 8           # Target feature dimension
  autoencoder:
    enabled: true
    epochs: 1000
    batch_size: 256
  cnn:
    enabled: true
    epochs: 20
    batch_size: 32
```

### Model Configuration
```yaml
# Classical models (Random Forest, XGBoost, etc.)
classical:
  enabled: true
  tune_model_params: true    # Enable hyperparameter tuning

# End-to-end deep learning
end-to-end:
  lstm:
    enabled: true
    epochs: 50
    batch_size: 32
  cnn:
    enabled: true
    epochs: 20
    batch_size: 32
  transformer:
    enabled: true
    epochs: 10
    batch_size: 4

advanced:
  verbose: true    # Detailed logging output
```

## Results

Results are automatically saved in the respective target directories specified in the config.