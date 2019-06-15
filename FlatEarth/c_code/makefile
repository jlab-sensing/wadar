# Target Compiler
#CC=gcc
#CC=gcc-linaro-6.5.0-2018.12-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
#CC=gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
CC=arm-mac-linux-gnueabihf/bin/arm-mac-linux-gnueabihf-gcc

# Relative paths in this dll project
NOVELDA_INC_DIR=usr/include/novelda
SALSA_INC_DIR=usr/include/flatearth
SALSA_LIB_DIR=usr/lib

# Compiler Options
CFLAGS=-Wall -O3 -I$(NOVELDA_INC_DIR) -I$(SALSA_INC_DIR)
#LDFLAGS=-L$(SALSA_LIB_DIR)/libRadarlib3.so -lRadarlib3 \
#-L$(SALSA_LIB_DIR)/libradarDSP.so -lradarDSP \
#-L$(SALSA_LIB_DIR)/cluttermap.so -lcluttermap \
#-L$(SALSA_LIB_DIR)/radarHelper.so -lradarHelper \
#-L$(SALSA_LIB_DIR)/anchoHelper.so -lanchoHelper
LDFLAGS=-Llib -Llib/arm-linux-gnueabihf -L$(SALSA_LIB_DIR) \
-l:libusb-1.0.so.0 -l:libjson-c.so.2 -l:libudev.so.1 \
-lRadarlib3 \
-lradarDSP \
-lcluttermap \
-lradarHelper \
-lcayenneHelper \
-lchipotleHelper \
-lanchoHelper

all: frameLogger.o frameViewer.o temp_test.o
	$(CC) $(CFLAGS) frameLogger.o $(LDFLAGS) -o FrameLogger
	$(CC) $(CFLAGS) frameViewer.o $(LDFLAGS) -o FrameViewer
	$(CC) $(CFLAGS) temp_test.o $(LDFLAGS) -o ReadTemp

frameLogger.o: frameLogger.c
	$(CC) -lrt -std=gnu99 -Wall -g -O3 $(CFLAGS) -c frameLogger.c $(LDFLAGS)

frameLoggerChipotle.o: frameLoggerChipotle.c
	$(CC) -lrt -std=gnu99 -Wall -g -O3 $(CFLAGS) -c frameLoggerChipotle.c $(LDFLAGS)

frameViewer.o: frameViewer.c
	$(CC) -lrt -std=gnu99 -Wall -g -O3 $(CFLAGS) -c frameViewer.c $(LDFLAGS)

temp_test.o: temp_test.c
	$(CC) -lrt -std=gnu99 -Wall -g -O3 $(CFLAGS) -c temp_test.c $(LDFLAGS)

clean:
	rm -rf *.o
	rm frameLoggerChipotle
	rm frameLogger
	rm frameViewer
	rm readTemp

deploy:
	scp frameLogger root@192.168.7.2:~/FlatEarth/Demos/Common/FrameLogger
	scp frameLoggerChipotle root@192.168.7.2:~/FlatEarth/Demos/Chipotle/FrameLogger 
