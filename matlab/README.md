# MATLAB Code for Radar Interface

Welcome to the MATLAB code repository for interfacing with radar(s). This codebase primarily interacts with FlatEarth libraries, demo code, and the WADAR scripts.

## Directory Structure

- **Legacy:** Contains obsolete MATLAB code, mainly for the X4 radar.
- **misc:** Directory for miscellaneous MATLAB scripts and tools that may not be actively used.
- **salsaMain.m:** Main MATLAB script for capturing new data or analyzing saved captures.
- **salsaPlot.m:** MATLAB script for loading radar data from a binary file.
- **salsaLoad.m:** MATLAB script for generating plots from processed radar data (usually called by salsaMain).
- **soil_moisture.m:** MATLAB script for converting Teros 12 RAW measurements and radar bin measurements to soil moisture.
- **demo.m:** Automated script used in the seminar "Enabling Sustainable Sensor Networks with Microbe-powered RF Backscatter" by Dr. Colleen Josephson. [Watch the seminar here](https://www.youtube.com/watch?v=WrSQfxIoFWw).
- **wadar.m:** Calculates the volumetric water content (vwc) of the soil from the radar capture. Used for automated data processing.
- **wadarTagTest.m:** Captures radar frames to test tag Signal-to-Noise Ratio (SNR) and displays the Fourier transform of the capture. Used for automated data processing.
- **wadarTemplateCapture.m:** Captures the template capture for correlation analysis. Used for automated data processing.
- **wadarAirCapture.m:** Captures the air capture for peak bin comparison. Used for automated data processing.

Additionally, there are files and directories related to FlatEarth example code or functions called by the WADAR scripts.