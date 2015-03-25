/*
This object is a led pivot. There are LED markers on it.
The position of LED markers and the position of the tip in it's own coordinate system 
are all stored. The read(string) function will read data from file provided.
*/
#ifndef BODY_H
#define BODY_H
#include "Eigen/Core"
#include "Eigen/LU"
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
using namespace Eigen;

class Body{

public:
	Vector3d* markers; // LED markers positions
	Vector3d tip; // tip position
	int num_markers; //number of markers
	Body(void);
	bool read(string);  //read data from a point
	~Body(void);
private:

};

#endif