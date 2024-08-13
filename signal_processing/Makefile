OBJS	= salsa.o
SOURCE	= salsa.c
HEADER	= salsa.h
OUT	= salsaTest
CC	 = gcc
FLAGS	 = -g -c -Wall
LFLAGS	 = 

all: $(OBJS)
	$(CC) -g $(OBJS) -o $(OUT) $(LFLAGS)

salsa.o: salsa.c
	$(CC) $(FLAGS) salsa.c 


clean:
	rm -f $(OBJS) $(OUT)