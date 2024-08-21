# Signal Processing Scripts for Chipotle Radar

This repository contains scripts to capture and process radar frames using the Chipotle radar. The primary use cases include measuring soil moisture content and testing WaDAR backscatter tags.

## Getting Started

### Prerequisites

- **Make**: Ensure `make` is installed on your system to compile the necessary binaries.
- **FFTW3**: Install the FFTW3 library for Fourier transforms:
  ```bash
  sudo apt-get install libfftw3-dev
  ```
- **Python**: Required for generating plots using `plotCaptureData.py`.
  - **Matplotlib**: Install via pip:
    ```bash
    pip install matplotlib
    ```
  - **NumPy**: Install via pip:
    ```bash
    pip install numpy
    ```

### Installation

1. Clone the repository.
2. Navigate to the project directory.

### Building the Project

Run the following command to compile the scripts:

```bash
make
```

## Usage

After building the project, you can run one of the following commands based on your use case:

### Measuring Soil Moisture Content

```bash
./wadar wadar -s <fullDataPath> -b <airFramesName> -t <trialName> -f <tagHz> -c <frameCount> -n <captureCount> -d <tagDepth>
```

```bash
./wadar wadarAirCapture -s <fullDataPath> -b <airFramesName> -f <tagHz> -c <frameCount> -n <captureCount>
```

```bash
./wadar wadarTwoTag -s <localDataPath> -t <trialName> -f <tag1Hz> -g <tag2Hz> -c <frameCount> -n <captureCount> -d <tagDiff>
```

### Testing the Tag

```bash
./wadar wadarTagTest -s <fullDataPath> -b <airFramesName> -t <trialName> -f <tagHz> -c <frameCount> -n <captureCount>
```

### Plotting the Results

After running `wadarTagTest`, you can generate plots using the following command:

```bash
python plotCaptureData.py
```

### Parameters

- `-s <fullDataPath>`: Path where local data will be stored.
- `-b <airFramesName>`: Name of the air frames file.
- `-t <trialName>`: Name of the trial.
- `-f <tagHz>`: Frequency of the tag (Hz).
- `-g <tag2Hz>`: Frequency of the second tag (Hz).
- `-c <frameCount>`: Number of frames to process.
- `-n <captureCount>`: Number of captures to perform.
- `-d <tagDepth> or <tagDiff>`: Depth of the tag or distance between two tags (m).

## Examples

### Example 1: Measuring Soil Moisture Content

```bash
./wadar wadar -s /data/moisture -b airFrames001 -t trialA -f 20000 -c 200 -n 10 -d 0.1
```

### Example 2: Testing the Tag

```bash
./wadar wadarTagTest -s /data/tagTest -b airFrames002 -t trialB -f 2000 -c 200 -n 10
```

### Example 3: Plotting the Results

```bash
python plotCaptureData.py
```
