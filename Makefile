.PHONY: all

all: clean libdump1090.so

clean:
	rm -f *.o

libdump1090.so: *.h *.c
	gcc -c -ggdb -fPIC dump1090.c -o dump1090.o
	gcc dump1090.o -ggdb -shared -o libdump1090.so -lrtlsdr

