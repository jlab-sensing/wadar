# Water Radar (WaDAR)

> __Note:__  
> README last updated February 2025

## Overview

**WaDAR** (Water Radar) is an open-source project that enables low-cost, high-accuracy soil moisture sensing using backscatter tags**. The system combines in-ground passive backscatter tags with above-ground radars to achieve wireless soil moisture monitoring with fine-grain resolution.

This project is based on the research published in [Low-cost In-ground Soil Moisture Sensing with Radar Backscatter Tags](https://doi.org/10.1145/3460112.3472326) and aims to make precision agriculture more affordable and scalable for farmers worldwide.

![wadar](wadar.png)

## System Components

The system consists of three main components:

| Component | Description | 
| --- | --- |
| [Backscatter Tag](https://github.com/jlab-sensing/wadar/tree/master/03_backscatter_tag) | A passive backscatter tag buried underground |
| [Radar](https://github.com/jlab-sensing/wadar/tree/master/02_uwb) | An above-ground UWB impulse radar |
| [Signal Processing Code](https://github.com/jlab-sensing/wadar/tree/master/01_dsp) | Processes radar signals for soil moisture estimation |


## File structure of repository

The following paths display a the high-level structure and description of each folder. Each individual folder contains a README with more information.

```
├── 00_documentation        # Lab journal documentation 
├── 01_dsp                  # C and MATLAB signal processing pipelines required to process radar signals for soil moisture estimation
├── 02_uwb                  # Software and hardware for the Chipotle UWB radar
├── 03_backscatter_tag      # Software and hardware for the passive backscatter tag
├── 04_data                 # Instructions on accessing radar frame data
├── 05_misc                 # Teros 12 software for comparison and shutdown script
├── 06_legacy               # Software for old hardware and old WaDAR project(s)
```

## Support

For issues relating to software and hardware, please create an issue in this repository. 

<!--## Contributing-->

<!--See [CONTRIBUTING.md](./CONTRIBUTING.md).-->

<!--## Code of Conduct-->

<!--This project adheres to-->
<!--[Contributor Covenant](https://www.contributor-covenant.org).-->
<!--See [Code of Conduct](./CODE_OF_CONDUCT.md) for a local copy.-->

<!--## License-->

<!--Code in this repository is licensed under the MIT License unless specified in the file header. See @ref LICENSE for full document.-->

## Maintainers

- [Colleen Josephson](mailto:cjosephson@ucsc.edu)
- [Eric Vetha](mailto:evetha@ucsc.edu)

## Citation
If you use this project in your research, please cite:

> C. Josephson, M. Kotaru, S. Katti, K. Winstein, R. Chandra,  
> "Low-cost In-ground Soil Moisture Sensing with Radar Backscatter Tags,"  
> *COMPASS '21*, June 28 - July 2, 2021, Virtual Event, Australia.  
> [DOI: 10.1145/3460112.3472326](https://doi.org/10.1145/3460112.3472326)

<!-- 
## Dependencies
The software is compatible with any Unix-based OS such as OSX 10.X or Linux. We have the following additional software dependencies:
- a recent MATLAB release ([here](https://drive.google.com/drive/u/1/folders/1ysOlEd1t2GFOKYOxBg3uzE3v5n-13ok5) is our group network MATLAB license if you don't already have one)
- RNDIS and FTDI drivers for ethernet and serial over USB to communicate with the radar
  	
   For OSX radar, install the network and serial drivers in step 2 of this site: http://beagleboard.org/getting-started. You might get an error about the developer being unknown, this is expected and the software is fine to install. To get around it, find the package(s) in finder and then ctrl-click on them and select open. 

   After installing, reboot and test by plugging in the radar, then opening command line and executing `ssh root@192.168.7.2`. You should be logged in to a terminal as root@beaglebone. You may get a warning like this, especially if you have tried a few different radars:
   
   `@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@`  
   `@    WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!     @`  
   `@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@`  
   
   This just means that you have SSH'd into another device that has used the same internal IP address. To solve this problem, use the terminal to edit this file: `/Users/<your_username>/.ssh/config` (create it if it doesn't exist). Add these two lines:
   
   `Host 192.168.7.2`  
   `  StrictHostKeyChecking no`  

   To test the serial connection, minicom is recommended. You may need to install homebrew to do this if you don’t have it already: `brew install minicom`. Then, once minicom is installed, test the connection by doing `minicom -D /dev/tty.usbmodem<XXXX>`. Fill in the XXXX with the appropriate device number. You can find it by using the terminal to look in the  /dev directory and grepping for /tty.usbmodem. Minicom will open, and it will display a message like this:  
   `Welcome to minicom 2.7.1`  
   `OPTIONS:`  
   `Compiled on May 17 2017, 15:29:14.
   `Port /dev/tty.usbmodem1413, 14:36:28`

   `Press Meta-Z for help on special keys`
   
   Press enter, and you should be logged into the radar and presented with this prompt: `root@beaglebone:~#`

- Arduino IDE plus the TeensyDuino add-on software for the teros12 and backscatter tag software
- gnuplot, to compile graphs
- latex to compile written documents (optional)

## Structure

The software for this system is divided into three key subsystems: radar software, backscatter software and commercial sensor software. 

The software for the backscatter tag and commercial sensor are pretty simple, basically just arduino files. You probably will not need to modify these very often.

The software for the radar is more complicated. The radars we use have two hardware components: the radar chip itself, and a Beagle Bone Black embedded linux board (BBB for short). We bought them from FlatEarth Sensing, and they named the radar development kit 'Chips and Salsa'; the chips refer to the radar chip and the salsa refers to their software library that collects data from the chip. Also worth mentioning is that the radars are all named after a type of pepper (ancho, cayenne and chipotle). Guess they like Mexican food.

The 'Salsa' software runs on the BBB. We have a modified version of some of their frameLogger code that we use to get our data, its in `/FlatEarth/c_code`. The BBB is a resource constrained device, so to get the maximum possible frame rate we do all the radar processing using MATLAB on a different device. That code lives in the matlab directory.

## Getting started

- Make sure all the prerequisites are installed
- Install all linked git submodules
```bash
git submodule update --init --recursive
```

- Make sure you can access the radar via `ssh root@192.168.7.2`.
- Go to the FlatEarth/c_code directory and make sure you can compile the frameLogger code and that it runs on the radar
- Go to the matlab directory and test that the salsaMain.m code works (check out Example 1 in the code comments at the top)

Congrats, you've successfully done a radar capture and processed it!

- Test with the tag

  Attach the radar to it's mount and point it towards the backscatter tag.  For best results, make sure the radar is between 20 and 50cm away from the top of the tag and that it's not underground for this initial test. In the plots, make sure you see a big peak when the tag is on and no peak when the tag is off. 
  
  IMPORTANT: some of the antennas (e.g. Vivaldi) have very strong polarization, so if you don't see any peak when the tag is on try rotating either the radar or the tag by 90 degrees. Also make sure you understand the radiation pattern of the type of antenna that is attached to the radar to ensure the direction of maximum gain is pointed towards the tag.
  
- Try out the soil_moisture.m program
  
  Now you can try burying the tag and using soil_moisture.m to compare the results against the commercial sensor. More detailed instructions coming soon (TODO). -->
