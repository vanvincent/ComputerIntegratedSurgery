#include "CaliObject.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <Eigen/Core>
using namespace Eigen;
using namespace std;

CaliObject::CaliObject(void){}

int CaliObject::reademmarkers(char* filename){

	ifstream infile(filename);
	
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}

	cout<<"Reading EM-Markers on a calibration object..."<<endl;

	int Nd,Na,Nc;
	char temp;
	string dummy;
	infile>>Nd>>temp>>Na>>temp>>Nc;
	getline(infile,dummy);

	emmarker_size = Nc;
	emmarker = new Marker[Nc];

	for(int i=0;i<Nd+Na;i++)
		getline(infile,dummy);

	for(int i=0;i<Nc;i++){
		double dx,dy,dz;
		infile>>dx>>temp>>dy>>temp>>dz;
		emmarker[i].pos[0] = dx;
		emmarker[i].pos[1] = dy;
		emmarker[i].pos[2] = dz;
	}
	
	infile.close();
	return 1;
}

int CaliObject::readoptmarkers(char* filename){

	ifstream infile(filename);
	
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}

	cout<<"Reading Optical-Markers on a calibration object..."<<endl;

	int Nd,Na,Nc;
	char temp;
	string dummy;
	infile>>Nd>>temp>>Na>>temp>>Nc;
	getline(infile,dummy);

	optmarker_size = Na;
	opticalmarker = new Marker[Na];

	for(int i=0;i<Nd;i++)
		getline(infile,dummy);

	for(int i=0;i<Na;i++){
		double dx,dy,dz;
		infile>>dx>>temp>>dy>>temp>>dz;
		opticalmarker[i].pos[0] = dx;
		opticalmarker[i].pos[1] = dy;
		opticalmarker[i].pos[2] = dz;
	}
	
	infile.close();
	return 1;
}

void CaliObject::write(char* filename){

  ofstream outfile;
  outfile.open (filename);

  outfile << "Optical markers:"<<endl;
  for(int i=0;i<optmarker_size;i++){
	  outfile <<opticalmarker[i].pos[0]<<"  "<<opticalmarker[i].pos[1]<<"  "<<opticalmarker[i].pos[2]<<endl;
  }

  outfile << "EM markers:"<<endl;
  for(int i=0;i<emmarker_size;i++){
	  outfile <<emmarker[i].pos[0]<<"  "<<emmarker[i].pos[1]<<"  "<<emmarker[i].pos[2]<<endl;
  }
  
  outfile.close();
}

CaliObject::~CaliObject() {
   // Deallocate the memory that was previously reserved
   if (opticalmarker)
      delete[] opticalmarker;

   if (emmarker)
      delete[] emmarker;
}