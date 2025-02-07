# Ultra-wideband Radar "Chipotle"

## Overview

This directory contains all necessary software and hardware for the UWB used in our sensing system. 

```
├── FlatEarth               # Software components for UWB radar
├── chipotle                # Hardware components for UWB radar
├── shutdown                # Shutdown script to safely shutdown the radar software
```

## Dependencies

The software is compatible with any Unix-based OS such as OSX 10.X or Linux. We have the following additional software dependencies:
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
- gnuplot, to compile graphs

## Structure

The radars we use have two hardware components: the radar chip itself, and a Beagle Bone Black embedded linux board (BBB for short). We bought them from FlatEarth Sensing, and they named the radar development kit 'Chips and Salsa'; the chips refer to the radar chip and the salsa refers to their software library that collects data from the chip. Also worth mentioning is that the radars are all named after a type of pepper (ancho, cayenne and chipotle). Guess they like Mexican food.

The 'Salsa' software runs on the BBB. We have a modified version of some of their frameLogger code that we use to get our data, its in `/FlatEarth/c_code`. The BBB is a resource constrained device, so to get the maximum possible frame rate we do all the radar processing using MATLAB on a different device. That code lives in the matlab directory.

## Getting started

- Make sure all the prerequisites are installed
- Make sure you can access the radar via `ssh root@192.168.7.2`.
- Go to the FlatEarth/c_code directory and make sure you can compile the frameLogger code and that it runs on the radar

Congrats, you've successfully done a radar capture and processed it!

- Test with the tag

  Attach the radar to it's mount and point it towards the backscatter tag.  For best results, make sure the radar is between 20 and 50cm away from the top of the tag and that it's not underground for this initial test. In the plots, make sure you see a big peak when the tag is on and no peak when the tag is off. 
  
  IMPORTANT: some of the antennas (e.g. Vivaldi) have very strong polarization, so if you don't see any peak when the tag is on try rotating either the radar or the tag by 90 degrees. Also make sure you understand the radiation pattern of the type of antenna that is attached to the radar to ensure the direction of maximum gain is pointed towards the tag.