CFLAGS = -O3 -Wall -pedantic -Iinclude --std=c99

ifeq ($(shell uname), Darwin)
	DRAWFLAGS = -framework Cocoa -framework OpenGL -lglfw
else
	DRAWFLAGS = `pkg-config --cflags --libs libglfw` -lGLU
endif

all: voxl samples

samples: objviewer benchmark

voxl: src/*.c
	cc -c $(CFLAGS) -o voxl.o src/voxl.c
	cc -c $(CFLAGS) -o tribox.o src/tribox.c 

objviewer: voxl samples/objviewer.c
	cc $(CFLAGS) -o objviewer voxl.o tribox.o samples/objviewer.c $(DRAWFLAGS)

benchmark: voxl samples/benchmark.c
	cc $(CFLAGS) -o benchmark voxl.o tribox.o samples/benchmark.c $(DRAWFLAGS)

clean:
	rm -f *.o objviewer benchmark