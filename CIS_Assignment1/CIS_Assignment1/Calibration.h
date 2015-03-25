#ifndef CALIBRATION_INCLUDED
#define CALIBRATION_INCLUDED
#include <stdio.h>
#include <Eigen/Core>
#include "Marker.h"
#include <Eigen/LU> 
#include "Pivot.h"
using namespace Eigen;

class Calibration {
public:
	MatrixXd pivotcalibration(Pivot&);
};

#endif