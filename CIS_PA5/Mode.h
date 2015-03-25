/*
This object is a mode consist of an original mesh and statitical atlas. the positions of triangle vertex are stored here
All the points are in some CI image coordiantes
*/
#ifndef MODE_H
#define MODE_H

#include "Eigen/Core"
#include "Triangle.h"
#include "Mesh.h"
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

class Mode{
public:
	Mesh original; //the original mesh
	Mesh* statistical; //the average mode and displacement
	int num_m; 
	int num_vertices;
	int num_triangles;

	Mode(void); //constructor
	bool readmodefile(string filename); //reading function that read mesh information
	void construct_mesh(string filename);//function that build all the modes
	~Mode(void); //destructor

};

#endif