/*
This is object is a triangle, with 3 vertices information
*/
#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "Eigen/Core"
#include "Eigen/LU"
#include <iostream>
using namespace std;
using namespace Eigen;

class Triangle{

public:
	Vector3d vertex[3]; //three vertices

	Triangle(void);
	Triangle(Vector3d a,Vector3d b,Vector3d c); //initialization
	double distance(const Vector3d tar); //this function compute the shortest distance from a point to the triangle
	Vector3d closest_point(const Vector3d target);// this function returen the closest point

private:
	Vector3d ProjectOnSegment(Vector3d,Vector3d,Vector3d);//helper function for computing closest point
	
};

#endif