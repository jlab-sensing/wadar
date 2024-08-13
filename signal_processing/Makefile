OBJS	= salsa.o proc.o utils.o
SOURCE	= salsa.c proc.c utils.c
HEADER	= salsa.h proc.h utils.h
OUT	= test
CC	 = gcc
FLAGS	 = -g -c -Wall
LFLAGS	 = 

all: $(OBJS)
	$(CC) -g $(OBJS) -o $(OUT) $(LFLAGS) -lm

salsa.o: salsa.c
	$(CC) $(FLAGS) salsa.c -lm

proc.o: proc.c
	$(CC) $(FLAGS) proc.c -lm


clean:
	rm -f $(OBJS) $(OUT)