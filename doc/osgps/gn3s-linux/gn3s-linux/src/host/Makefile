INCLUDES=-I. -I/usr/local/include
CPP=g++ -O3 -Wall -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64
CC=gcc -O3 -Wall -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64

all: host

host: GN3SFull_new.cpp GN3SFull.h dll_hj.c fusb_linux.cc fusb_linux.h fusb.cc fusb.h fusb_sysconfig_linux.cc TimeStuff.cpp TimeStuff.h
	$(CPP) $(INCLUDES) -c GN3SFull_new.cpp -o GN3SFull.o
	$(CPP) $(INCLUDES) -c dll_hj.c -o dll_hj.o
	$(CPP) $(INCLUDES) -c fusb_linux.cc -o fusb_linux.o
	$(CPP) $(INCLUDES) -c fusb.cc -o fusb.o
	$(CPP) $(INCLUDES) -c fusb_sysconfig_linux.cc -o fusb_sysconfig_linux.o
	$(CPP) $(INCLUDES) -c TimeStuff.cpp -o TimeStuff.o
	$(CPP) $(INCLUDES) -o gn3s-full dll_hj.o GN3SFull.o fusb_linux.o fusb.o fusb_sysconfig_linux.o TimeStuff.o -lusb -lpthread

install:
	mv gn3s-full ../../bin

clean: 
	rm -f *.o
