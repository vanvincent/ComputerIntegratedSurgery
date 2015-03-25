#include "EMtracker.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <Eigen/Core>
using namespace std;

EMtracker::EMtracker(void){}

int EMtracker::readoptmarkers(string filename){

	ifstream infile(filename);
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}
	cout<<"Reading Optical Markers on EM Tracker..."<<endl;

	int Nd,Na,Nc;
	char temp;
	string dummy;
	infile>>Nd>>temp>>Na>>temp>>Nc;
	getline(infile,dummy);

	optmarker_size = Nd;
	opticalmarker = new Marker[Nd];

	for(int i=0;i<Nd;i++){
		double dx,dy,dz;
		infile>>dx>>temp>>dy>>temp>>dz;
		opticalmarker[i].pos[0] = dx;
		opticalmarker[i].pos[1] = dy;
		opticalmarker[i].pos[2] = dz;
	}
	
	infile.close();
	return 1;
}

int EMtracker::read(string filename){
	ifstream infile(filename);
	
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}
	cout<<"EM tracker tracking..."<<endl;

	int Nd,Na,Nc,Nf;
	char temp;
	string dummy;
	infile>>Nd>>temp>>Na>>temp>>Nc>>temp>>Nf;
	getline(infile,dummy);

	frame_size = Nf;

	data = new Reading[Nf];

	for(int j = 0; j<Nf; j++){
		
		data[j].size = Nc;
		data[j].marker = new Marker[Nc];

		for(int i=0;i<Nd+Na;i++)
			getline(infile,dummy);

		for(int i=0;i<Nc;i++){
			double dx,dy,dz;
			infile>>dx>>temp>>dy>>temp>>dz;
			data[j].marker[i].pos[0] = dx;
			data[j].marker[i].pos[1] = dy;
			data[j].marker[i].pos[2] = dz;
			
		}
		getline(infile,dummy);
	}
	infile.close();
	return 1;
}

void EMtracker::write(char* filename){
  ofstream outfile;
  outfile.open (filename);
  outfile << "EMtracker:"<<endl;
  outfile << "Optical markers on embase:"<<endl;
  for(int i=0;i<optmarker_size;i++){
	  outfile <<opticalmarker[i].pos[0]<<"  "<<opticalmarker[i].pos[1]<<"  "<<opticalmarker[i].pos[2]<<endl;
  }
  outfile << "EM reading"<<endl;
  for(int i=0;i<this->frame_size;i++){
	  outfile << "Reading "<<i<<endl;
	  int size = this->data[i].size;
	  for(int j=0;j<size;j++)
		  outfile <<this->data[i].marker[j].pos[0]<<"  "<<this->data[i].marker[j].pos[1]<<"  "<<this->data[i].marker[j].pos[2]<<endl;
  }
  outfile.close();
}

EMtracker::~EMtracker(){

	if(data){
		delete[] data;
	}
	if(opticalmarker)
		delete[] opticalmarker;

}