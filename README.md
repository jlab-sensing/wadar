# Radar Soil Sensing Project
## TODO: cross-compiler for OSX

The sensing system consists of four main components: the backscatter tag, the radar, the matlab code that processes the radar signal, and the teros12 commercial sensor that we use to verify the radar measurements. 

The software is compatible with any Unix-based OS such as OSX 10.X or Linux. We have the following additional software dependencies:
- a recent MATLAB release 
- RNDIS and FTDI drivers for ethernet and serial over USB to communicate with the radar
  	
   For OSX radar, install the network and serial drivers in step 2 of this site: http://beagleboard.org/getting-started. You might get an error about the developer being unknown, this is expected and the software is fine to install. To get around it, find the package(s) in finder and then ctrl-click on them and select open. 

   After installing, reboot and test by plugging in the radar, then opening command line and executing `ssh root@192.168.7.2`. You should be logged in to a terminal as root@beaglebone.

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

*Below is an overview of the directory structure. There are additional README files within most of the subdirectories.*

backscatterTag: code for the radar backscatter tag prototype

FlatEarth: code that runs on the Flat Earth Ancho, Cayenne and Chipotle radars

legacy: old code that is no longer used

matlab: matlab code, mainly for interfacing with the radar

teros: code for interfacing with the Teros12 commercial soil sensor

writing: latex files for papers, gnuplot and dat files for graphs

