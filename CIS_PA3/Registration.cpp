#include "Registration.h"
#include "Eigen/SVD"
#include "Eigen/Eigenvalues" 
#include <iostream>

using namespace Eigen;
using namespace std;

Matrix4d Registration::arun(Vector3d* group1,Vector3d* group2,int size){

	Vector3d centroid1;
	Vector3d centroid2;
	centroid1 <<0,0,0;
	centroid2 <<0,0,0;
	Vector3d* newgroup1 = new Vector3d[size];
	Vector3d* newgroup2 = new Vector3d[size];
	Matrix3d H = Matrix3d::Zero();
	Matrix3d X;

	//step1
	for(int i=0;i<size;i++){
		centroid1 = centroid1 + group1[i];
	}
	centroid1 /= size;
	for(int i=0;i<size;i++){
		centroid2  = centroid2 + group2[i];
	}
	centroid2 /= size;
	//step2
	for(int i=0;i<size;i++){
		newgroup1[i] = group1[i] - centroid1;
	}
	for(int i=0;i<size;i++){
		newgroup2[i] = group2[i] - centroid2;
	}

	//step3
	for(int i=0; i<size; i++){
		H = H + newgroup1[i] * ( newgroup2[i].transpose());
	}
	JacobiSVD<MatrixXd> svd(H, ComputeThinU | ComputeThinV);
	Matrix3d U = svd.matrixU();
	Matrix3d V = svd.matrixV(); 
	X =V *( U.transpose());
	//setp4
	double determinant = X.determinant();
	if((int)determinant != 1){
		Matrix3d V = svd.matrixV();
		Vector3d V3 = V.block<3,1>(0,2);
		V3 = V3 * -1;
		V.block<3,1>(0,2) = V3;
		X = V*U.transpose();
	}

	Vector3d T = centroid2 - X * centroid1;
	Matrix4d re = Matrix4d::Identity();
	re.block<3,3>(0,0) = X;
	re.block<3,1>(0,3) = T;

	delete[] newgroup1;
	delete[] newgroup2;
	return re;
}


Matrix4d Registration::horn(Vector3d* group1,Vector3d* group2,int size){

	Vector3d centroid1;
	Vector3d centroid2;
	centroid1 <<0,0,0;
	centroid2 <<0,0,0;
	Vector3d* newgroup1 = new Vector3d[size];
	Vector3d* newgroup2 = new Vector3d[size];
	Matrix3d H = Matrix3d::Zero();
	

	//step1
	for(int i=0;i<size;i++){
		centroid1 = centroid1 + group1[i];
	}
	centroid1 /= size;
	for(int i=0;i<size;i++){
		centroid2  = centroid2 + group2[i];
	}
	centroid2 /= size;
	//step2
	for(int i=0;i<size;i++){
		newgroup1[i] = group1[i] - centroid1;
	}
	for(int i=0;i<size;i++){
		newgroup2[i] = group2[i] - centroid2;
	}
	//step3
	for(int i=0; i<size; i++){
		H = H + newgroup1[i] *( newgroup2[i].transpose());
	}

	Vector3d delta(H(1,2)-H(2,1),H(2,0)-H(0,2),H(0,1)-H(1,0));
	Matrix4d G = Matrix4d::Zero();
	G(0,0) = H.trace();
	G.block<1,3>(0,1) = delta.transpose();
	G.block<3,1>(1,0) = delta;
	G.block<3,3>(1,1) = H + H.transpose() -(Matrix3d::Identity() )* (H.trace());


	EigenSolver<MatrixXd> es(G);
	VectorXd q = es.eigenvectors().col(0).real();
	
	
	Matrix3d X;
	X<< 1-2*q(2)*q(2) - 2*q(3)*q(3),  2*q(1)*q(2)-2*q(3)*q(0),   2*q(1)*q(3)+2*q(2)*q(0),
		2*q(1)*q(2)+2*q(3)*q(0),      1-2*q(1)*q(1)-2*q(3)*q(3), 2*q(2)*q(3) - 2*q(1)*q(0),
		2*q(1)*q(3)-2*q(2)*q(0),      2*q(2)*q(3)+2*q(1)*q(0),   1-2*q(1)*q(1)-2*q(2)*q(2);

	Vector3d T = centroid2 - X * centroid1;
	Matrix4d re = Matrix4d::Identity();
	re.block<3,3>(0,0) = X;
	re.block<3,1>(0,3) = T;
	
	delete[] newgroup1;
	delete[] newgroup2;
	
	return re;
}