# MATLAB code

This MATLAB code interfaces with the radar(s), mostly FlatEarth libraries or demo code. 

## Directories and files

- Legacy: old MATLAB code we don't use anymore, mainly for the X4 radar
- misc: miscellaneous matlab scripts and tools we probably won't use again
- NoveldaDDC.m: library code for digital downconversion of the radar RF signal to a baseband signal
- salsaMain.m: use this file to capture new data or analyze saved captures
- salsaData.m: TODO
- salsaPlot.m: loads radar data from a binary file (captured from frameLogger.c on BBB)
- salsaLoad.m: makes plots from processed radar data (usually called by salsaMain)
- soil_moisture.m: code to convert Teros 12 RAW measurements and radar bin measurements to soil moisture
