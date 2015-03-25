#include "Calibration.h"
#include "Registration.h"
#include <Eigen/SVD>
#include <iostream>
using namespace std;

Calibration::Calibration(){
	max_x=-1;
	min_x=INT_MAX;
	max_y=-1;
	min_y=INT_MAX;
	max_z=-1;
	min_z=INT_MAX;
}

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

	JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
	MatrixXd re = svd.solve(B);

	delete[] g;
	delete[] F;
	return re;
}

Marker* Calibration::CorrectDistortion(Marker* q, Marker* u,int size,int degree){
	
	ScaleToBox_local(q,u,size);

	MatrixXd BM = Bernstein_matrix(u,size,degree);
	MatrixXd CM = BM * this->Bernstein_C;
	
	
	for(int i = 0; i < size; i++){
		u[i].pos(0) = CM(i,0);
		u[i].pos(1) = CM(i,1);
		u[i].pos(2) = CM(i,2);
		
	}
	return u;
}

void Calibration::ScaleToBox(Marker* q,Marker* scaled,int size){
	

	for(int i = 0; i<size;i++){

		if(q[i].pos(0) > max_x)
			max_x = q[i].pos(0);
		if(q[i].pos(0)< min_x)
			min_x = q[i].pos(0);
		if(q[i].pos(1) > max_y)
			max_y = q[i].pos(1);
		if(q[i].pos(1)< min_y)
			min_y = q[i].pos(1);
		if(q[i].pos(2) > max_z)
			max_z = q[i].pos(2);
		if(q[i].pos(2)< min_z)
			min_z = q[i].pos(2);

	}
	
	for(int i = 0;i<size;i++){
		scaled[i].pos(0) = (q[i].pos(0) - min_x)/(max_x - min_x);
		scaled[i].pos(1) = (q[i].pos(1) - min_y)/(max_y - min_y);
		scaled[i].pos(2) = (q[i].pos(2) - min_z)/(max_z - min_z);
		
	}
	
}

void Calibration::ScaleToBox_local(Marker* q,Marker* scaled,int size){
		
	for(int i = 0;i<size;i++){
		scaled[i].pos(0) = (q[i].pos(0) - min_x)/(max_x - min_x);
		scaled[i].pos(1) = (q[i].pos(1) - min_y)/(max_y - min_y);
		scaled[i].pos(2) = (q[i].pos(2) - min_z)/(max_z - min_z);
	}
	
}

MatrixXd Calibration::Bernstein_matrix(Marker* distorted, int size, int degree){

	MatrixXd F(size,(degree+1)*(degree+1)*(degree+1));
	int count=0;
	
	for(int s=0;s<size;s++){
		for(int i=0;i<degree+1;i++){
			for(int j=0;j<degree+1;j++){
				for(int k=0;k<degree+1;k++){
					F(s,count++) = calculate_bernstein_value(distorted[s],i,j,k,degree);
				}
			}
		}
		count=0;
	}

	return F;
}

double Calibration::calculate_bernstein_value(Marker val,int i,int j,int k, int degree){

	double x = val.pos(0);
	double y = val.pos(1);
	double z = val.pos(2);

	return (combination(degree,i)*pow(x,i)*pow((1-x),(degree-i)))*
		   (combination(degree,j)*pow(y,j)*pow((1-y),(degree-j)))*
	       (combination(degree,k)*pow(z,k)*pow((1-z),(degree-k)));

}

int Calibration::combination(int d,int c){
	return factorial(d)/(factorial(c)*factorial((d-c)));
}

int Calibration::factorial( int v){

	int factorial;
	for (int i = 0; i <= v; i++){
	
		if (i == 0)
		factorial = 1;
		else
		factorial = factorial * i;
	}
	return factorial;

}

MatrixXd Calibration::Bernstein(Marker* expected, Marker* distorted, int size , int degree){
	
	Marker* u = new Marker[size];
	ScaleToBox(distorted,u,size);

	MatrixXd F;
	F = Bernstein_matrix(u,size,degree);
	MatrixXd P(size,3);
	for(int s=0;s<size;s++){
		for(int i=0;i<3;i++){
			P(s,i)=expected[s].pos(i);
			
		}
	}
	
	MatrixXd Coeff;
	JacobiSVD<MatrixXd> svd(F, ComputeThinU | ComputeThinV);
	Coeff = svd.solve(P);
	Bernstein_C = Coeff;
	
	delete[] u;
	return Coeff;
}

