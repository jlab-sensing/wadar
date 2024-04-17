# FrameLogger Documentation

[frameLogger.c - Wadar GitHub Repository](https://github.com/jlab-sensing/wadar/blob/master/FlatEarth/c_code/frameLogger.c)

### INTRO

frameLogger.c is the source code for a capture program that runs locally on the BBB. This code was developed because it allows us to capture frames faster and at a more even frame rate than using the MATLAB connector.

### PREREQUISITES 

Before compiling the code, make sure that the BBB can access your computer via ssh. This is so it can copy radar captures to your local machine for live processing. On OSX you'll need to turn on Remote Login within the Sharing section of System Preferences. Then, you need to allow publickey authentication from the BBB. To do so, edit or create this file on your Mac: `~/.ssh/authorized_keys`. Then add this line:

`from="192.168.7.2" ssh-rsa AAA...sshkeyjibberish...`

Everything after `from"192.168.7.2"` is the contents of the id_rsa.pub file on the BBB, which is located in the `~/.ssh` directory. So ssh into the BBB and copy that file's contents into your authorized_keys folder. 

Then test that you can ssh into your computer FROM the BBB by doing `ssh <your_username>@192.168.7.1` *while you are already ssh'd into the BBB*. You should get a response like this:

`Last login: Wed Jun 19 15:55:06 2019 from 192.168.7.2
DN0a24a1e6:~ cjoseph$ `

### COMPILATION 

To compile the code, run 'make frameLogger' in this directory. It will use the linaro cross-compiler to generate an
arm-compatible binary. Then run `make deploy` to copy the code over to the BBB (the radar needs to be plugged in to your computer).

### USAGE 

To run the code locally on the BBB, ssh in to root@192.168.7.2 and cd to `/root/FlatEarth/Demos/Common/FrameLogger`

Here is an example usage:

`./frameLogger -s ../data/captureSettings -l ../data/captureData -n 2000 -r 3 -f 200 -t cayenne -c cjoseph@192.168.7.1:/Users/cjoseph/Documents/research/radar/matlab/data`

-s provides the path to the capture settings folder
-l is the path and file prefix for the capture dump
-n is the number of frames per run
-r is the number of runs, which dictates the number of dumps
-f is the frame rate
-t is the type of the radar (cayenne, ancho, chipotle)
-c is the copy path, the directory on a local or remote computer to transfer the files to

The above command will produce 3 different 10-second captures at
200fps and dump them into the data directory. The dump format is
binary.

More detailed usage instructions are available in the source code. 

To ensure that the code runs at the most even possible frame rate, you
can prefix the program execution with `ionice -c 1 -n 0 nice -n -20`.

### POST-PROCESSING

You can run the frameLogger to generate the capture and post-process using MATLAB later, or you can use the MATLAB scripts with the "wadar" prefix to capture and process them immediately.
