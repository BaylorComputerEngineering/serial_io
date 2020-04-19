serial_io: serial_io.c
	gcc -c -o serial_io.o  serial_io.c
	ar rcs libserial_io.a serial_io.o
	cp libserial_io.a /usr/lib
	cp serial_io.h /usr/include
	gcc example_app.c -lserial_io  -oexample_app	