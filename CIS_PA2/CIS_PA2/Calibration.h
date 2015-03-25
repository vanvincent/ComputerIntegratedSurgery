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
	double max_x,min_x;
	double max_y,min_y;
	double max_z,min_z;
	Marker* CorrectDistortion(Marker*,Marker*, int,int);
	MatrixXd Bernstein_C;
	MatrixXd Bernstein(Marker*,Marker*,int,int);
	MatrixXd pivotcalibration(Pivot&);
	Calibration();
private:
	void ScaleToBox_local(Marker* q,Marker* scaled,int size);
	void ScaleToBox(Marker*,Marker*,int);
	MatrixXd Bernstein_matrix(Marker*,int,int);
	double calculate_bernstein_value(Marker val,int i,int j,int k, int degree);
	int combination(int d,int c);
	int factorial( int v);
	
};

#endif