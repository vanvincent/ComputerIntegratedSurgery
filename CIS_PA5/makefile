CC=g++
CFLAGS=-c -Wall

all: PA5

PA5: main.o Mesh.o Triangle.o Body.o OPTtracker.o Registration.o Mode.o
	g++ main.o Mesh.o Triangle.o Body.o OPTtracker.o Registration.o Mode.o -o PA5

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

Mode.o: Mode.cpp
	$(CC) $(CFLAGS) Mode.cpp

clean:
	rm -rf *.o PA5