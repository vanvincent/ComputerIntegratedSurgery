/*
This object is a mesh consist of a lot of triangles. the positions of triangle vertex are stored here
All the points are in some CI image coordiantes
*/
#ifndef MESH_H
#define MESH_H

#include "Eigen/Core"
#include "Triangle.h"
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

class Mesh{
public:

	int num_vertices; 
	int num_triangles;
	Vector3d* vertices; //the vertices list read from file
	Vector3i* indices; //the indices list of triangle read from file
	Triangle* triangle_list; // the triangles contructed using vertices and indices information

	Mesh(void); //constructor
	
	bool readmeshfile(string filename); //reading function that read mesh information
	~Mesh(void); //destructor
	void construct_triangle(); //private function that construct triangle from the vertices list and the indices list

};

#endif