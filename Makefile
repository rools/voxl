CFLAGS = -O3 -Wall -pedantic -Iinclude --std=c99
LIBS = -lpng
LINKFLAGS =

ifeq ($(shell uname), Darwin)
	DRAWLIBS = -framework Cocoa -framework OpenGL -lglfw
else
	DRAWLIBS = `pkg-config --libs glfw3 gl` -lm -lpthread
endif

all: voxl samples

samples: objviewer benchmark

voxl: src/*.c
	cc -c $(CFLAGS) -o voxl.o src/voxl.c
	cc -c $(CFLAGS) -o tribox.o src/tribox.c
	cc -c $(CFLAGS) -o polyconv.o src/polyconv.c

objviewer: voxl samples/objviewer.c
	cc $(CFLAGS) $(LINKFLAGS) -o objviewer voxl.o tribox.o polyconv.o samples/objviewer.c $(LIBS) $(DRAWLIBS)

benchmark: voxl samples/benchmark.c
	cc $(CFLAGS) $(LINKFLAGS) -o benchmark voxl.o tribox.o polyconv.o samples/benchmark.c $(LIBS) $(DRAWLIBS)

clean:
	rm -f *.o objviewer benchmark
