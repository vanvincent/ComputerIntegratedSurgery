#include "Mode.h"

Mode::Mode(void){
	num_vertices = num_m = 0;
}

//read mode files 
bool Mode::readmodefile(string filename){
	//open file, if fail return false and stop program
	ifstream infile(filename.c_str());
	if(!infile.is_open()){
		cout<<"Error openning file."<<endl;
		return false;
	}
	//dispaly someting
	cout<<"Reading mode:"<<filename<<endl;
	//declare some intermediate variables



	char sep[2]="=";
	char temp[20];
	char str1[20];
	char str2[20];
	infile>>temp>>str1>>str2;
	char* vertices;
	char* modes;
	int ver, mod;
	vertices= strtok(str1,sep);
	vertices= strtok(NULL,sep);
	modes=strtok(str2,sep);
	modes=strtok(NULL,sep);
	ver = atoi(vertices);
	mod = atoi(modes);

	string dummy;
	char t;
	num_m = mod;
	statistical = new Mesh[num_m+1];

	getline(infile,dummy);
	for(int k=0;k<num_m+1;k++){

		getline(infile,dummy);
		cout<<dummy<<endl;
		statistical[k].vertices = new Vector3d[num_vertices]; //allocate vertices
		statistical[k].num_vertices = num_vertices;
		statistical[k].num_triangles = num_triangles;
		//looping to read all infomation
		for(int i=0;i<num_vertices;i++){
			double dx,dy,dz;
			infile>>dx>>t>>dy>>t>>dz;
			getline(infile,dummy);
			statistical[k].vertices[i](0) = dx;
			statistical[k].vertices[i](1) = dy;
			statistical[k].vertices[i](2) = dz;
		}
		
		statistical[k].indices = new Vector3i[num_triangles];
		//looping to read all indices info from original mesh
		for(int i=0;i<num_triangles;i++){
			
			statistical[k].indices[i](0) = original.indices[i](0);
			statistical[k].indices[i](1) = original.indices[i](1);
			statistical[k].indices[i](2) = original.indices[i](2);
			
		}
		
		statistical[k].construct_triangle();
	
	}
	//close file
	infile.close();

	return true;
}

void Mode::construct_mesh(string filename){
	original.readmeshfile(filename);
}


Mode::~Mode(void){
	//clear used memory
	if(statistical)
		delete[] statistical;
}
