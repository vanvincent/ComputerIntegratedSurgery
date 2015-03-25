/*
CIS Programming Assignment1
Authors: Han Xiao and Kunal Saluja
Date:10-29-2014
*/
#include <iostream>
#include <Eigen/Core>
#include "CaliObject.h"
#include "OpticalTracker.h"
#include "EMTracker.h"
#include "Registration.h"
#include "Pivot.h"
#include "Calibration.h"
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace Eigen;
using namespace std;

int main()
{

	//delcaration of calsses
	CaliObject obj;
	OpticalTracker opttrack;
	EMtracker emtrack;
	Pivot empivot;
	Calibration cali;
	Registration reg;

	ofstream outfile;
	outfile.open ("OUTPUT/pa1-debug-d-our-output_new.txt");
	
	//reading datas
	obj.reademmarkers("INPUT/pa1-debug-d-calbody.txt");  //read calibration object markers info
	obj.readoptmarkers("INPUT/pa1-debug-d-calbody.txt");
	emtrack.readoptmarkers("INPUT/pa1-debug-d-calbody.txt"); //read em tracker markers info
	opttrack.read("INPUT/pa1-debug-d-calreadings.txt");  //reading multiple frames from em tracker and optical tracker
	emtrack.read("INPUT/pa1-debug-d-calreadings.txt");
	empivot.emread("INPUT/pa1-debug-d-empivot.txt"); //read information from em pivot

	
	outfile<<obj.emmarker_size<<", "<<opttrack.frame_size<<"pa1-debug-d-our-output_new.txt"<<endl;
	//emtrack.write("test2.txt");
	//obj.write("test.txt");
	//opttrack.write("test3.txt");
	//pivot.write("pivotreading.txt");

	
	//problem5
	outfile<< fixed;
	outfile<<setprecision(2);
	MatrixXd dimple_em = cali.pivotcalibration(empivot);
	outfile<<"  "<<dimple_em(3)<<",   "<<dimple_em(4)<<",   "<<dimple_em(5)<<endl;

	//problem6
	OpticalTracker opttrack2; 
	Pivot opticalpivot;
	opttrack2.readem("INPUT/pa1-debug-d-optpivot.txt");
	opticalpivot.opticalread("INPUT/pa1-debug-d-optpivot.txt");

	for(int i=0;i<opticalpivot.frame_size;i++){
		//Matrix4d Fd = reg.arun(emtrack.opticalmarker,opttrack2.data[i].emled,emtrack.optmarker_size,opttrack2.data[i].emled_size);
		Matrix4d Fd = reg.horn(emtrack.opticalmarker,opttrack2.data[i].emled,emtrack.optmarker_size);
		Matrix3d R = Fd.inverse().block<3,3>(0,0);
		Vector3d T = Fd.inverse().block<3,1>(0,3);
		for(int j=0;j<opticalpivot.marker_size;j++){
			opticalpivot.data[i].marker[j].pos = R * opticalpivot.data[i].marker[j].pos + T;
		}
	}

	MatrixXd dimple_opt = cali.pivotcalibration(opticalpivot);
	outfile<<"  "<<dimple_opt(3)<<",   "<<dimple_opt(4)<<",   "<<dimple_opt(5)<<endl;
	
	//problem4
	for(int i = 0;i< opttrack.frame_size;i++){
		
		//Matrix4d Fd = reg.arun(emtrack.opticalmarker,opttrack.data[i].emled,emtrack.optmarker_size,opttrack.data[i].emled_size);
		//Matrix4d Fa = reg.arun(obj.opticalmarker,opttrack.data[i].caliobjled,obj.optmarker_size,opttrack.data[i].caliobjled_size);
		Matrix4d Fd = reg.horn(emtrack.opticalmarker,opttrack.data[i].emled,emtrack.optmarker_size);
		Matrix4d Fa = reg.horn(obj.opticalmarker,opttrack.data[i].caliobjled,obj.optmarker_size);
		
		Matrix4d M = Fd.inverse() * Fa;
		Matrix3d R = M.block<3,3>(0,0);
		Vector3d T = M.block<3,1>(0,3);
		Marker* C_expected =new Marker[obj.emmarker_size];
		for(int i = 0; i< obj.emmarker_size;i++){
			C_expected[i].pos = R * obj.emmarker[i].pos + T;
			outfile<< fixed;
			outfile<<setprecision(2);
			outfile<<"  "<<C_expected[i].pos(0)<<",   "<<C_expected[i].pos(1)<<",   "<<C_expected[i].pos(2)<<endl; //output
		}
		
	}

	return 0;
}
