CC=g++
CFLAGS=-c -Wall

all: PA4

PA4: main.o Mesh.o Triangle.o Body.o OPTtracker.o Registration.o
	g++ main.o Mesh.o Triangle.o Body.o OPTtracker.o Registration.o -o PA4

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp

Mesh.o: Mesh.cpp
	$(CC) $(CFLAGS) Mesh.cpp

Triangle.o: Triangle.cpp
	$(CC) $(CFLAGS) Triangle.cpp

Body.o: Body.cpp
	$(CC) $(CFLAGS) Body.cpp

OPTtracker.o: OPTtracker.cpp
	$(CC) $(CFLAGS) OPTtracker.cpp

Registration.o: Registration.cpp
	$(CC) $(CFLAGS) Registration.cpp

clean:
	rm -rf *.o PA4