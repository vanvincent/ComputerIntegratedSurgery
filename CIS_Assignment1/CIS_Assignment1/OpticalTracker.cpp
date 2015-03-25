#include "OpticalTracker.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
using namespace std;

OpticalTracker::OpticalTracker(void){}

int OpticalTracker::read(char* filename){
	ifstream infile(filename);
	
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}

	cout<<"OpticalTracer tracking..."<<endl;

	int Nd,Na,Nc,Nf;
	char temp;
	string dummy;
	infile>>Nd>>temp>>Na>>temp>>Nc>>temp>>Nf;
	getline(infile,dummy);

	frame_size = Nf;

	data = new Reading[Nf];

	for(int j = 0; j<Nf; j++){

		data[j].emled_size = Nd ;
		data[j].caliobjled_size = Na; 
		data[j].emled = new Marker[Nd];
		data[j].caliobjled = new Marker[Na];

		for(int i=0;i<Nd;i++){
			double dx,dy,dz;
			infile>>dx>>temp>>dy>>temp>>dz;
			data[j].emled[i].pos[0] = dx;
			data[j].emled[i].pos[1] = dy;
			data[j].emled[i].pos[2] = dz;
		}

		for(int i=0;i<Na;i++){
			double dx,dy,dz;
			infile>>dx>>temp>>dy>>temp>>dz;
			data[j].caliobjled[i].pos[0] = dx;
			data[j].caliobjled[i].pos[1] = dy;
			data[j].caliobjled[i].pos[2] = dz;
		}
		getline(infile,dummy);
		for(int i=0;i<Nc;i++)
			getline(infile,dummy);

	}
	infile.close();
	return 1;
}

int OpticalTracker::readem(char* filename){
	ifstream infile(filename);
	
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}
	cout<<"OpticalTracer tracking pivot..."<<endl;

	int Nd,Nh,Nf;
	char temp;
	string dummy;
	infile>>Nd>>temp>>Nh>>temp>>Nf;
	getline(infile,dummy);

	frame_size = Nf;

	data = new Reading[Nf];

	for(int j = 0; j<Nf; j++){

		data[j].emled_size = Nd; 
		data[j].emled = new Marker[Nd];

		for(int i=0;i<Nd;i++){
			double dx,dy,dz;
			infile>>dx>>temp>>dy>>temp>>dz;
			data[j].emled[i].pos[0] = dx;
			data[j].emled[i].pos[1] = dy;
			data[j].emled[i].pos[2] = dz;
		}

		getline(infile,dummy);

		for(int i=0;i<Nh;i++)
			getline(infile,dummy);
	}
	infile.close();
	return 1;
}

void OpticalTracker::write(char* filename){

  ofstream outfile;
  outfile.open (filename);
  outfile << "Optical tracker reading"<<endl;
  for(int i=0;i<this->frame_size;i++){
	  outfile << "Reading "<<i<<endl;
	  int size = this->data[i].emled_size;
	  outfile << "em tracker_led reading"<<endl;
	  for(int j=0;j<size;j++)
		  outfile <<this->data[i].emled[j].pos[0]<<"  "<<this->data[i].emled[j].pos[1]<<"  "<<this->data[i].emled[j].pos[2]<<endl;
	  size = this->data[i].caliobjled_size;
	  outfile << "caliobj_led reading"<<endl;
	  for(int j=0;j<size;j++)
		  outfile <<this->data[i].caliobjled[j].pos[0]<<"  "<<this->data[i].caliobjled[j].pos[1]<<"  "<<this->data[i].caliobjled[j].pos[2]<<endl;
	  
  }
  outfile.close();
}

OpticalTracker::~OpticalTracker(){

	if(data){
		delete[] data;
	}

}