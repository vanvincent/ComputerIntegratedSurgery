#include "Calibration.h"
#include "Registration.h"
#include <Eigen/SVD>
#include <iostream>
using namespace std;

MatrixXd Calibration::pivotcalibration(Pivot& pivot){

	Marker* g=new Marker[pivot.marker_size];
	Matrix4d *F =new Matrix4d[pivot.frame_size];
	Registration reg;
	Vector3d G;
	G <<0,0,0;

	for(int i=0;i<pivot.marker_size;i++){
		G = G + pivot.data[0].marker[i].pos;
	}
	G /= pivot.marker_size;

	for(int i=0;i<pivot.marker_size;i++){
		g[i].pos = pivot.data[0].marker[i].pos - G;
	}

	for(int i=0;i<pivot.frame_size;i++){
		F[i] = reg.arun(g,pivot.data[i].marker,pivot.marker_size,pivot.marker_size);
		//F[i] = reg.horn(g,pivot.data[i].marker,pivot.marker_size);
	}

	MatrixXd A(3*pivot.frame_size,6);
	MatrixXd B(3*pivot.frame_size,1);

	for(int i=0;i<pivot.frame_size;i++){
		A.block<3,3>(i*3,0) = F[i].block<3,3>(0,0);
		A.block<3,3>(i*3,3) = -Matrix3d::Identity();
		B.block<3,1>(i*3,0) = - F[i].block<3,1>(0,3);
	}

	//MatrixXd re = (A.transpose()*A).inverse()* A.transpose() * B;
	JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
	MatrixXd re = svd.solve(B);

	delete[] g;
	delete[] F;

	return re;
}