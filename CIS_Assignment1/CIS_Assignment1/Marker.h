#ifndef MARKER_INCLUDED
#define MARKER_INCLUDED
#include <stdio.h>
#include <Eigen/Core>
using namespace Eigen;
class Marker {
	public:
		Vector3d pos;
		Marker(void);
		Marker(const double& x,const double& y,const double& z);
};

#endif