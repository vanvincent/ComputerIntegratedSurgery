#include "CT.h"
#include <fstream>
#include <iostream>
#include <string>
using namespace std;

int CT::read(string filename){

	ifstream infile(filename);
	
	if(infile ==NULL){
		cout<<"Error openning file."<<endl;
		return 0;
	}
	cout<<"CT reading..."<<endl;

	int Nb;
	char temp;
	string dummy;
	infile>>Nb;
	getline(infile,dummy);

	
	marker_size = Nb;
	marker = new Marker[Nb];

	for(int i=0;i<Nb;i++){
		double dx,dy,dz;
		infile>>dx>>temp>>dy>>temp>>dz;
		marker[i].pos[0] = dx;
		marker[i].pos[1] = dy;
		marker[i].pos[2] = dz;
	}
	getline(infile,dummy);
	
	infile.close();
	return 1;
}

CT::~CT(){
	if(marker)
		delete[] marker;
}