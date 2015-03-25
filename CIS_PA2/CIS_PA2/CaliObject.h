#ifndef CALI_INCLUDED
#define CALI_INCLUDED
#include <stdio.h>
#include <string>
#include "Marker.h"
using namespace std;

class CaliObject {
public:
	Marker* opticalmarker;
	Marker* emmarker;
	int optmarker_size;
	int emmarker_size;
	CaliObject(void);
	int reademmarkers(string);
	int readoptmarkers(string);
	void write(char*);
	~CaliObject(); 
};

#endif