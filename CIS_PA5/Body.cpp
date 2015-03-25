#include "Body.h"
//initialize the body,which are the two pivot read by em tracker
Body::Body(void){
	num_markers = 0;
}
//reading body from file
bool Body::read(string filename){

	ifstream infile(filename.c_str());
	if(!infile.is_open()){
		cout<<"Error openning file."<<endl;
		return false;
	}
	//dispaly someting
	cout<<"Reading file:"<<filename<<endl;
	//declare some intermediate variables
	
	string dummy;
	infile>>num_markers;
	getline(infile,dummy);
	markers = new Vector3d[num_markers]; //allocate markers
	//looping to read all infomation
	for(int i=0;i<num_markers;i++){
		double dx,dy,dz;
		infile>>dx>>dy>>dz;
		markers[i](0) = dx;
		markers[i](1) = dy;
		markers[i](2) = dz;
	}
	
	//looping to read all infomation
	
	double dx,dy,dz;
	infile>>dx>>dy>>dz; //reading
	tip(0) = dx;
	tip(1) = dy;
	tip(2) = dz;
		
	
	//close file
	infile.close();
	return true;
}
//clear memories
Body::~Body(void){
	
	if(markers){
		delete[] markers;
	}

}