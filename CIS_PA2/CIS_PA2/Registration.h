#ifndef REG_INCLUDED
#define REG_INCLUDED
#include <stdio.h>
#include <Eigen/Core>
#include "Marker.h"
#include <Eigen/LU> 
using namespace Eigen;

class Registration {
	public:
		Matrix4d arun(Marker*,Marker*,int,int);
		Matrix4d horn(Marker*,Marker*,int);
};

#endif