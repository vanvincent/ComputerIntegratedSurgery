#pragma once
#ifndef CT_INCLUDED
#define CT_INCLUDED
#include <stdio.h>
#include <Eigen/Core>
#include "Marker.h"
#include <Eigen/LU> 
#include <string>
using namespace std;
class CT {
public:
	
	Marker * marker;
	int marker_size;
	
	int read(string);
	void write(char*);
	~CT();

};
#endif