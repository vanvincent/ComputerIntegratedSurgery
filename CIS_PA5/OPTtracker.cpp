#include "OPTtracker.h"
#include "Body.h"

OPTtracker::OPTtracker(void){
	samples = BodyA_marker = BodyB_marker = Dummy_marker=0;
}
//parsing funciton
bool OPTtracker::read(string filename,int A,int B){

	ifstream infile(filename.c_str());
	
	if(!infile.is_open()){
		cout<<"Error openning file."<<endl;
		return 0;
	}
	cout<<"OPT tracker reading:"<<filename<<endl;

	int total;
	char temp;
	string dummy;
	infile>>total>>temp>>samples;
	getline(infile,dummy);

	int BodyA_marker = A;
	int BodyB_marker = B;
	int Dummy_marker = total - (A+B);
	data = new Reading[samples];

	for(int j = 0; j<samples; j++){
		
		data[j].BodyA = new Vector3d[BodyA_marker];
		data[j].BodyB = new Vector3d[BodyB_marker];
		data[j].Dummy = new Vector3d[Dummy_marker];

		for(int i=0;i<A;i++){
			double dx,dy,dz;
			infile>>dx>>temp>>dy>>temp>>dz; //reading
			data[j].BodyA[i](0) = dx;
			data[j].BodyA[i](1) = dy;
			data[j].BodyA[i](2) = dz;
		}

		for(int i=0;i<B;i++){
			double dx,dy,dz;
			infile>>dx>>temp>>dy>>temp>>dz; //reading
			data[j].BodyB[i](0) = dx;
			data[j].BodyB[i](1) = dy;
			data[j].BodyB[i](2) = dz;
		}

		for(int i=0;i<Dummy_marker;i++){
			double dx,dy,dz;
			infile>>dx>>temp>>dy>>temp>>dz; //reading
			data[j].Dummy[i](0) = dx;
			data[j].Dummy[i](1) = dy;
			data[j].Dummy[i](2) = dz;
		}

	}
	infile.close();
	return true;
}

OPTtracker::~OPTtracker(void){
	if(data)
		delete[] data;
}
