# Target Compiler
CC=gcc

# Relative paths in this dll project
NOVELDA_INC_DIR=/usr/include/novelda
SALSA_INC_DIR=/usr/include/flatearth
SALSA_LIB_DIR=/usr/lib
UTILITY_DIR=../../Common/

# Compiler Options
CFLAGS=-Wall -I$(NOVELDA_INC_DIR) -I$(SALSA_INC_DIR)
LDFLAGS=-L$(SALSA_LIB_DIR)/libRadarlib3.so -lRadarlib3 \
-L$(SALSA_LIB_DIR)/radarDSP.so -lradarDSP \
-L$(SALSA_LIB_DIR)/radarHelper.so -lradarHelper \
-L$(SALSA_LIB_DIR)/chipotleHelper.so -lchipotleHelper

all: radarDemo.o utility.o normalize.o
	$(CC) $(CFLAGS) radarDemo.o utility.o normalize.o $(LDFLAGS) -o RadarDemo

radarDemo.o: radarDemo.c $(UTILITY_DIR)/inc/utility.h $(UTILITY_DIR)/inc/normalize.h
	$(CC) $(CFLAGS) -c radarDemo.c $(LDFLAGS)

utility.o: $(UTILITY_DIR)/src/utility.c $(UTILITY_DIR)/inc/utility.h
	$(CC) $(CFLAGS) -I$(UTILITY_DIR)/inc -c $(UTILITY_DIR)/src/utility.c $(LDFLAGS)

normalize.o: $(UTILITY_DIR)/src/normalize.c $(UTILITY_DIR)/inc/normalize.h
	$(CC) $(CFLAGS) -I$(UTILITY_DIR)/inc -c $(UTILITY_DIR)/src/normalize.c $(LDFLAGS)

clean:
	rm -rf *.o
	rm RadarDemo
