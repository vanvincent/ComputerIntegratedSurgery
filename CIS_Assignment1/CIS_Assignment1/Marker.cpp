#include "Marker.h"
using namespace Eigen;
Marker::Marker(void){
	pos[0]=pos[1]=pos[2]=0;
};
Marker::Marker(const double& x,const double& y,const double& z){
	pos[0]=x; pos[1]=y; pos[2]=z;			
};
