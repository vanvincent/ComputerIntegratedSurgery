#ifndef EMPIVOT_INCLUDED
#define EMPIVOT_INCLUDED
#include <stdio.h>
#include <Eigen/Core>
#include "Marker.h"
#include <Eigen/LU> 
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

	int opticalread(char*);
	int emread(char*);
	void write(char*);
	~Pivot();

};

#endif