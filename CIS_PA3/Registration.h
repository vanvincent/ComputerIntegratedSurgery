/*
This is the registraion object that containing Arun's and Horn's method.
Same as PA1 and PA2
We do not include explainations of the algorithms in commments for these two functions.
We already discussed the algorithms in PA1 and PA2, please refer to PA1 and PA2
*/
#ifndef REG_INCLUDED
#define REG_INCLUDED
#include <stdio.h>
#include "Eigen/Core"
#include "Eigen/LU"

using namespace Eigen;

class Registration {
	public:
		Matrix4d arun(Vector3d*,Vector3d*,int);
		Matrix4d horn(Vector3d*,Vector3d*,int);
};

#endif