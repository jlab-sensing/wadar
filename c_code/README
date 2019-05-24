### INTRO

frameLogger.c is the source code for a capture program that runs
locally on the BBB. This code was developed because it allows us to
capture frames faster and at a more even frame rate than using the
MATLAB connector.

### COMPILATION 

To compile the code locally, run 'make frameLogger' in this
directory. It will use the linaro cross-compiler to generate an
arm-compatible binary.

To compile the code on the BBB, copy the source file over to the BBB
to this directory:

/home/root/FlatEarth/Demos/Ancho/FrameLogger2

Then run 'make frameLogger' in that directory.

### INSTALLATION 

If you compiled the code on the BBB, the installation is
complete. Proceed to the USAGE section.

If you ran the compiler locally, you need to copy the generated
frameLogger binary over to the BBB in the directory
/home/root/FlatEarth/Demos/Ancho/FrameLogger2.

### USAGE 

Detailed usage instructions are available in the source code. Here is
an example command:

./frameLogger -s ../data/captureSettings -l ../data/captureData -r 3 -n 2000 -f 200

-s provides the path to the capture settings folder
-l is the path and file prefix for the capture dump
-r is the number of runs, which dictates the number of dumps
-n is the number of frames per run
-f is the frame rate

The above command will produce 3 different 10-second captures at
200fps and dump them into the data directory. The dump format is
binary.

To ensure that the code runs at the most even possible frame rate, you
can prefix the program execution with `ionice -c 1 -n 0 nice -n -20`.

### POST-PROCESSING

To process the capture data, copy the data files to a computer with
MATLAB and use the saveData_Ancho.m file to read in the files.