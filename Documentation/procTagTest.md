# procTagTest.m

[procTagTest.m - Wadar GitHub Repository](https://github.com/jlab-sensing/wadar/blob/master/matlab/procTagTest.m)

## procTagTest

Function to process radar frames for backscatter tag testing.

### Inputs:

- **localDataPath**: Path of radar frames storage.
- **captureName**: Name of radar frames file.

### Outputs:

Displays SNR and peak magnitude results.

### Example:

```matlab
procTagTest('data_folder', 'radar_capture_1.mat');
```

### Description:

This function processes radar frames to analyze backscatter tags. It loads radar frames from the specified local data path and capture name, performs baseband conversion, computes the tag Fourier transform (FT), calculates SNR, identifies the peak magnitude, and displays the results.

The function also plots the tag FT and the FT of all peak bins to visualize the data.
