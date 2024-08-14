OBJS	= salsa.o proc.o utils.o
SOURCE	= salsa.c proc.c utils.c
HEADER	= salsa.h proc.h utils.h
OUT	= test
CC	 = gcc
FLAGS	 = -g -c -Wall
LFLAGS	 = 

all: $(OBJS)
	$(CC) -g $(OBJS) -o $(OUT) $(LFLAGS) -lfftw3f -lfftw3 -lm

salsa.o: salsa.c
	$(CC) $(FLAGS) salsa.c -lfftw3f -lfftw3 -lm

proc.o: proc.c
	$(CC) $(FLAGS) proc.c -lfftw3f -lfftw3 -lm


clean:
	rm -f $(OBJS) $(OUT)