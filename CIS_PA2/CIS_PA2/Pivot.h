#ifndef EMPIVOT_INCLUDED
#define EMPIVOT_INCLUDED
#include <stdio.h>
#include <Eigen/Core>
#include "Marker.h"
#include <Eigen/LU> 
#include <string>
using namespace std;
using namespace Eigen;

class Pivot {
public:
	
	Marker * marker;
	int marker_size;
	int frame_size;

	struct Reading{
	public:
		Marker *marker;
	};

	Reading* data;

	int opticalread(string);
	int emread(string);
	void write(char*);
	~Pivot();

};

#endif