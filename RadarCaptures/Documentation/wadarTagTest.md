# procTagTest.m

[wadarTagTest.m - Wadar GitHub Repository](https://github.com/jlab-sensing/wadar/blob/master/matlab/wadarTagTest.m)

## procTagTest

Function to capture and process radar frames for backscatter tag testing. This script was used to capture the majority of the captures in this repository.

### Inputs:

- **DisplayAllFrames**: If true, displays each capture as the captures are taken
- **localDataPath**: Path of radar frames storage.
- **tagName**: Name of tag for file naming purposes
- **trialName**: Trial name for file naming purposes
- **captureCount**: Number of captures desired

### Output:

None

### Example:

```matlab
wadarTagTest(true, '/home/ericdvet/jlab/wadar/matlab/data/RFSwitchTesting', 'PE', 'WetSoil', 3);
```

### Description:

This function first captures then processes radar frames to analyze backscatter tags. It loads radar frames from the specified local data path and capture name, performs baseband conversion, computes the tag Fourier transform (FT), calculates SNR, identifies the peak magnitude, and displays the results. The results compose of the median and mean results of all captures.

The function also plots the final capture's tag FT and the FT of all peak bins to visualize the data. 
