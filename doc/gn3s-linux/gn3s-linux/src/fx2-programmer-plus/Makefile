CC=g++ -pipe -g -Wall -O2

all: fx2-programmer test1

test1: fx2-programmer
	@echo "---------------- FX2 PROGRAMMER -----------------------"
	@./fx2-programmer || true

fx2-programmer: fx2-programmer.c
	$(CC) fx2-programmer.c -lusb -o fx2-programmer	

install:
	mv fx2-programmer ../../bin/

