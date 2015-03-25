#ifndef EM_INCLUDED
#define EM_INCLUDED
#include <stdio.h>
#include "Marker.h"
#include <string>
using namespace std;

class EMtracker {
public:
	EMtracker(void);
	int optmarker_size;
	int frame_size;

	struct Reading{
	public:
		int size;
		Marker *marker;
	};

	Reading* data;
	Marker* opticalmarker;

	int read(string);
	int readoptmarkers(string);
	void write(char*);
	~EMtracker();
};

#endif