/*
CIS Programming Assignment2
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
#include "CT.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <omp.h>
#include <string>

using namespace Eigen;
using namespace std;


void Error_eval(string,string,string);

int main()
{
	//delcaration of calsses
	CaliObject obj;
	OpticalTracker opttrack;
	EMtracker emtrack;
	Pivot empivot;
	Calibration cali;
	Registration reg;
	Pivot empivot_fiducials;
	CT ct;
	Pivot empivot_nav;
	OpticalTracker opttrack2; 
	Pivot opticalpivot;
	
	int option;
	string data_type;
	cout<<"For debug data set press 1, For unknown data set press 2:"<<endl;
	cin>>option;

	if(option == 1)
		data_type="debug";

	else if(option == 2)
		data_type="unknown";
	else{
		cout<<"Wrong input.";
		return 0;
	}

	char data_set;
	cout<<"Please enter data set lable (a,b,c...)"<<endl;
	cin>>data_set;

	//reading datas
	obj.reademmarkers("INPUT/pa2-" + data_type + "-"+data_set+"-calbody.txt");   //read calibration object markers info
	obj.readoptmarkers("INPUT/pa2-" + data_type + "-"+data_set+"-calbody.txt");
	emtrack.readoptmarkers("INPUT/pa2-" + data_type + "-"+data_set+"-calbody.txt"); //read em tracker markers info
	opttrack.read("INPUT/pa2-" + data_type + "-"+data_set+"-calreadings.txt");  //reading multiple frames from em tracker and optical tracker
	emtrack.read("INPUT/pa2-" + data_type + "-"+data_set+"-calreadings.txt");
	empivot.emread("INPUT/pa2-" + data_type + "-"+data_set+"-empivot.txt"); //read information from em pivot
	empivot_fiducials.emread("INPUT/pa2-" + data_type + "-"+data_set+"-em-fiducialss.txt"); //em pivot reading when touching fiducials
	ct.read("INPUT/pa2-" + data_type + "-"+data_set+"-ct-fiducials.txt"); //fiducial in CT coordinate
	empivot_nav.emread("INPUT/pa2-" + data_type + "-"+data_set+"-EM-nav.txt"); //navigation points 
	opttrack2.readem("INPUT/pa2-" + data_type + "-"+data_set+"-optpivot.txt");  //optical pivot info, for redoing pivot calibration
	opticalpivot.opticalread("INPUT/pa2-" + data_type + "-"+data_set+"-optpivot.txt");

	ofstream outfile,outfile2;
	outfile.open ("OUTPUT/pa2-" + data_type + "-"+data_set+"-our-output1.txt");
	outfile<<obj.emmarker_size<<", "<<opttrack.frame_size<<" pa2-" + data_type + "-"+data_set+"-our-output2.txt"<<endl;
	outfile2.open ("OUTPUT/pa2-" + data_type + "-"+data_set+"-our-output2.txt");
	outfile2<<empivot_nav.frame_size<<", "<<"pa2-" + data_type + "-"+data_set+"-our-output2.txt"<<endl;

	//*********************Problem 1 and 2*****************************************//
	//computer C_expected
	Marker* C_expected =new Marker[obj.emmarker_size * emtrack.frame_size];

	for(int j = 0;j<emtrack.frame_size;j++){
		//Matrix4d Fd = reg.arun(emtrack.opticalmarker,opttrack.data[j].emled,emtrack.optmarker_size,opttrack.data[j].emled_size);
		//Matrix4d Fa = reg.arun(obj.opticalmarker,opttrack.data[j].caliobjled,obj.optmarker_size,opttrack.data[j].caliobjled_size);
		Matrix4d Fd = reg.horn(emtrack.opticalmarker,opttrack.data[j].emled,emtrack.optmarker_size);
		Matrix4d Fa = reg.horn(obj.opticalmarker,opttrack.data[j].caliobjled,obj.optmarker_size);
		Matrix4d M = Fd.inverse() * Fa;
		Matrix3d R = M.block<3,3>(0,0);
		Vector3d T = M.block<3,1>(0,3);
		for(int i = 0; i< obj.emmarker_size;i++){
			C_expected[i+j*obj.emmarker_size].pos = R * obj.emmarker[i].pos + T;
		}
	}
	//loading the distorted EM readings
	Marker* C_distorted =new Marker[obj.emmarker_size * emtrack.frame_size];
	for(int j = 0;j<emtrack.frame_size;j++){
		for(int i = 0; i< obj.emmarker_size;i++){
			C_distorted[i+j*obj.emmarker_size].pos = emtrack.data[j].marker[i].pos;
		}
	}
    //calculate Bernstein coefficent and store in the object cali
	cali.Bernstein(C_expected,C_distorted,obj.emmarker_size * emtrack.frame_size,5);
	
	
	//*********************Problem 3*****************************************//
	//redo EM pivot calibration
	//correction every frame first
	for(int i=0;i<empivot.frame_size;i++){
		Marker* corrected_frame = new Marker[empivot.marker_size];
		cali.CorrectDistortion(empivot.data[i].marker, corrected_frame ,empivot.marker_size,5);
		for(int j =0;j<empivot.marker_size;j++){
			empivot.data[0].marker[j].pos(0) = corrected_frame[j].pos(0);
			empivot.data[0].marker[j].pos(1) = corrected_frame[j].pos(1);
			empivot.data[0].marker[j].pos(2) = corrected_frame[j].pos(2);
		}
		delete[] corrected_frame;
	}

	MatrixXd dimple_em = cali.pivotcalibration(empivot);

	cout<<"******pivot info*******"<<endl;
	cout<< fixed;
	cout<<setprecision(2);
	cout<<"  "<<dimple_em(3)<<",   "<<dimple_em(4)<<",   "<<dimple_em(5)<<endl;
	//output
	outfile<< fixed;
	outfile<<setprecision(2);
	outfile<<setw(8)<<dimple_em(3)<<","<<setw(9)<<dimple_em(4)<<","<<setw(9)<<dimple_em(5)<<endl;
	Vector3d dc;   //sotre dc for later use
	dc(0) = dimple_em(0);
	dc(1) = dimple_em(1);
	dc(2) = dimple_em(2);

	//redo optical pivot calibration
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
	cout<<"***opt pivot info"<<endl;
	cout<<"  "<<dimple_opt(3)<<",   "<<dimple_opt(4)<<",   "<<dimple_opt(5)<<endl;
	outfile<<setw(8)<<dimple_opt(3)<<","<<setw(9)<<dimple_opt(4)<<","<<setw(9)<<dimple_opt(5)<<endl;


	//correct distortion for every frame(in problem 2, but put here for generating output files)
	for(int j = 0;j<emtrack.frame_size;j++){
		Marker* corrected_frame = new Marker[obj.emmarker_size];
		cali.CorrectDistortion(emtrack.data[j].marker, corrected_frame ,obj.emmarker_size,5);
		for(int i = 0; i< emtrack.data[0].size;i++){
			outfile<< fixed;
			outfile<<setprecision(2);
			outfile<<setw(8)<<corrected_frame[i].pos(0) <<","<<setw(9)<<corrected_frame[i].pos(1)<<","<<setw(9)<<corrected_frame[i].pos(2)<<endl; //output
		}
		delete[] corrected_frame;
	}


	//*********************Problem 4 and 5*****************************************//
	//correction every frame first
	for(int i=0;i<empivot_fiducials.frame_size;i++){
		Marker* corrected_frame = new Marker[empivot_fiducials.marker_size];
		cali.CorrectDistortion(empivot_fiducials.data[i].marker, corrected_frame ,empivot_fiducials.marker_size,5);
		for(int j =0;j<empivot_fiducials.marker_size;j++){
			empivot_fiducials.data[i].marker[j].pos(0) = corrected_frame[j].pos(0);
			empivot_fiducials.data[i].marker[j].pos(1) = corrected_frame[j].pos(1);
			empivot_fiducials.data[i].marker[j].pos(2) = corrected_frame[j].pos(2);
		}
		delete[] corrected_frame;
	}

	//calculating tips positions respect to em tracker base (f_expected)
	Marker* f_expected =new Marker[empivot_fiducials.frame_size];

	Vector3d G;
	G <<0,0,0;
	Marker* g=new Marker[empivot.marker_size];

	for(int i=0;i<empivot.marker_size;i++){
		G = G + empivot.data[0].marker[i].pos;
	}
	G /= empivot.marker_size;
	for(int i=0;i<empivot.marker_size;i++){
		g[i].pos = empivot.data[0].marker[i].pos - G;
	}
	for(int j = 0;j< empivot_fiducials.frame_size;j++){
		Matrix4d Fd = reg.arun(g,empivot_fiducials.data[j].marker,empivot_fiducials.marker_size,empivot_fiducials.marker_size);
		//Matrix4d Fd = reg.horn(g,empivot_fiducials.data[j].marker,empivot_fiducials.marker_size);
		Matrix3d R = Fd.block<3,3>(0,0);
		Vector3d T = Fd.block<3,1>(0,3);
		f_expected[j].pos = R * dc + T;
	}
	//use f_expected and ct coordinate info to compute Freg
	//Matrix4d Freg = reg.arun(f_expected,ct.marker,empivot_fiducials.marker_size,empivot_fiducials.marker_size);
	Matrix4d Freg = reg.horn(f_expected,ct.marker,empivot_fiducials.marker_size);
	Matrix3d Rct = Freg.block<3,3>(0,0);
	Vector3d Tct = Freg.block<3,1>(0,3);

	//****************************Problem 6*************************************//
	//apply freg to nav data
	//correction every frame
	for(int i=0;i<empivot_nav.frame_size;i++){
		Marker* corrected_frame = new Marker[empivot_nav.marker_size];
		cali.CorrectDistortion(empivot_nav.data[i].marker, corrected_frame ,empivot_nav.marker_size,5);
		for(int j =0;j<empivot_nav.marker_size;j++){
			empivot_nav.data[i].marker[j].pos(0) = corrected_frame[j].pos(0);
			empivot_nav.data[i].marker[j].pos(1) = corrected_frame[j].pos(1);
			empivot_nav.data[i].marker[j].pos(2) = corrected_frame[j].pos(2);
		}
		delete[] corrected_frame;
	}
	//calculating tips positions respect to em tracker base(f_tip)
	Marker* f_tip =new Marker[empivot_nav.frame_size];

	cout<<"Final Output"<<endl;
	//perform a calibration 
	G <<0,0,0;
	Marker* g2=new Marker[empivot.marker_size];
	for(int i=0;i<empivot.marker_size;i++){
		G = G + empivot.data[0].marker[i].pos;
	}
	G /= empivot.marker_size;
	for(int i=0;i<empivot_nav.marker_size;i++){
		g2[i].pos = empivot.data[0].marker[i].pos - G;
	}
	for(int j = 0;j< empivot_nav.frame_size;j++){
		Matrix4d Fd = reg.arun(g2,empivot_nav.data[j].marker,empivot_nav.marker_size,empivot_nav.marker_size);
		//Matrix4d Fd = reg.horn(g2,empivot_nav.data[j].marker,empivot_nav.marker_size);
		Matrix3d R = Fd.block<3,3>(0,0);
		Vector3d T = Fd.block<3,1>(0,3);
		Vector3d f_tips_em = R * dc + T;
		f_tip[j].pos = Rct * f_tips_em + Tct;
		cout<< fixed;
		cout<<setprecision(2);
		cout<<"  "<<f_tip[j].pos(0)<<",   "<<f_tip[j].pos(1)<<",   "<<f_tip[j].pos(2)<<endl; //output
		outfile2<< fixed;
		outfile2<<setprecision(2);
		outfile2<<setw(8)<<f_tip[j].pos(0)<<","<<setw(9)<<f_tip[j].pos(1)<<","<<setw(9)<<f_tip[j].pos(2)<<endl; //output
	}


	outfile.close();
	outfile2.close();

	Error_eval("INPUT/pa2-" + data_type + "-"+data_set+"-output1.txt","OUTPUT/pa2-" + data_type + "-"+data_set+"-our-output1.txt","OUTPUT/pa2-" + data_type + "-"+data_set+"-error_analysis1.txt");
	Error_eval("INPUT/pa2-" + data_type + "-"+data_set+"-output2.txt","OUTPUT/pa2-" + data_type + "-"+data_set+"-our-output2.txt","OUTPUT/pa2-" + data_type + "-"+data_set+"-error_analysis2.txt");

	delete[] C_expected;
	delete[] C_distorted;
	delete[] f_expected;
	delete[] f_tip;
	delete[] g;
	delete[] g2;
	return 0;
}


void Error_eval(string file_a,string file_b,string out){

	ifstream infile1(file_a);
	ofstream outfile(out);

	if(infile1 ==NULL){
		cout<<"Error openning file."<<endl;
		return;
	}

	ifstream infile2(file_b);
	if(infile2 ==NULL){
		cout<<"Error openning file."<<endl;
		return;
	}

	int count = 0;
	char temp;
	string dummy;
	getline(infile1,dummy);
	getline(infile2,dummy);
	outfile<<dummy<<endl;
	outfile<<"Individual errors"<<endl;

	double dx,dy,dz,dx1,dy1,dz1,x_sum = 0,y_sum=0,z_sum=0;
	while(infile1>>dx>>temp>>dy>>temp>>dz && infile2>>dx1>>temp>>dy1>>temp>>dz1){
		outfile<<setw(9)<<dx1-dx<<setw(9)<<dy1-dy<<setw(9)<<dz1-dz<<endl;;
		x_sum+=abs(dx1-dx);
		y_sum+=abs(dy1-dy);
		z_sum+=abs(dz1-dz);
		count++;
	}

	x_sum/=count;
	y_sum/=count;
	z_sum/=count;

	outfile<<"*******Data analysis*******"<<endl;
	outfile<<"Total number of data are:"<<count<<endl;
	outfile<<"The average error(absolute) for X coordinate accross all data set is:"<<x_sum<<endl;
	outfile<<"The average error(absolute) for Y coordinate accross all data set is:"<<y_sum<<endl;
	outfile<<"The average error(absolute) for Z coordinate accross all data set is:"<<z_sum<<endl;

	infile1.close();
	infile2.close();
	outfile.close();
	return;
}