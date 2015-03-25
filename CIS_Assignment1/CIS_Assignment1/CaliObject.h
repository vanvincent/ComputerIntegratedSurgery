#ifndef CALI_INCLUDED
#define CALI_INCLUDED
#include <stdio.h>
#include "Marker.h"


class CaliObject {
public:
	Marker* opticalmarker;
	Marker* emmarker;
	int optmarker_size;
	int emmarker_size;
	CaliObject(void);
	int reademmarkers(char*);
	int readoptmarkers(char*);
	void write(char*);
	~CaliObject(); 
};

#endif