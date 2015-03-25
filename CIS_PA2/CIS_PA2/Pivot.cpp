#include "Pivot.h"
#include <fstream>
#include <iostream>
using namespace std;

int Pivot::emread(string filename){

	ifstream infile(filename);
	
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}
	cout<<"EM tracker tracking Pivot..."<<endl;

	int Ng,Nf;
	char temp;
	string dummy;
	infile>>Ng>>temp>>Nf;
	getline(infile,dummy);

	frame_size = Nf;
	marker_size = Ng;
	data = new Reading[Nf];
	
	for(int j = 0; j<Nf; j++){
		
		data[j].marker = new Marker[Ng];

		for(int i=0;i<Ng;i++){
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

int Pivot::opticalread(string filename){

	ifstream infile(filename);
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}
	cout<<"Optical pivot read by tracker..."<<endl;

	int Nd,Nh,Nf;
	char temp;
	string dummy;
	infile>>Nd>>temp>>Nh>>temp>>Nf;
	getline(infile,dummy);

	frame_size = Nf;
	marker_size = Nh;
	data = new Reading[Nf];
	
	for(int j = 0; j<Nf; j++){
		
		data[j].marker = new Marker[Nh];

		for(int i=0;i<Nd;i++)
			getline(infile,dummy);

		for(int i=0;i<Nh;i++){
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

void Pivot::write(char* filename){
  ofstream outfile;
  outfile.open (filename);
  outfile << "EM reading"<<endl;
  int size = this->marker_size;
  for(int i=0;i<this->frame_size;i++){
	  outfile << "Reading "<<i<<endl;
	  for(int j=0;j<size;j++)
		  outfile <<this->data[i].marker[j].pos[0]<<"  "<<this->data[i].marker[j].pos[1]<<"  "<<this->data[i].marker[j].pos[2]<<endl;
  }
  outfile.close();
}

Pivot::~Pivot(){
	if(data){
		delete[] data;
	}
}