/*
This is an optical tracker that read LED information
The structure is the same opttracker we used in PA1 and PA2
*/
#ifndef OPT_INCLUDED
#define OPT_INCLUDED
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include "Eigen/Core"
#include "Body.h"

using namespace std;
using namespace Eigen;

class OPTtracker {
public:
	
	int samples; //how many readings
	int BodyA_marker;
	int BodyB_marker;
	int Dummy_marker;

	struct Reading{
		Vector3d* BodyA;
		Vector3d* BodyB;
		Vector3d* Dummy;
	}; //reading including two bodies' readings and some unused dummy readings

	Reading* data; //pointer for reading data

	OPTtracker(void);
	bool read(string,int,int); //read data from file, and the number of led on each body
	~OPTtracker(void);
};

#endif