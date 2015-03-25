#include "Mesh.h"

Mesh::Mesh(void){
	num_vertices = num_triangles = 0;
}

void Mesh::construct_triangle(void){
	// allocate memory for triangles
	cout<<"Constructing mesh from triangles."<<endl;
	triangle_list = new Triangle[num_triangles];
	//construct triangle using vertices and indices info
	for(int i = 0; i<num_triangles ; i++){
		triangle_list[i] = Triangle(vertices[indices[i](0)],vertices[indices[i](1)],vertices[indices[i](2)]);
	}
}

bool Mesh::readmeshfile(string filename){
	//open file, if fail return false and stop program
	ifstream infile(filename.c_str());
	if(!infile.is_open()){
		cout<<"Error openning file."<<endl;
		return false;
	}
	//dispaly someting
	cout<<"Reading mesh:"<<filename<<endl;
	//declare some intermediate variables
	
	string dummy;
	infile>>num_vertices;
	vertices = new Vector3d[num_vertices]; //allocate vertices
	//looping to read all infomation
	for(int i=0;i<num_vertices;i++){
		double dx,dy,dz;
		infile>>dx>>dy>>dz;
		vertices[i](0) = dx;
		vertices[i](1) = dy;
		vertices[i](2) = dz;
	}
	//allocate indices 
	infile>>num_triangles;
	indices = new Vector3i[num_triangles];
	//looping to read all infomation
	for(int i=0;i<num_triangles;i++){
		int dx,dy,dz;
		infile>>dx>>dy>>dz; //reading
		indices[i](0) = dx;
		indices[i](1) = dy;
		indices[i](2) = dz;
		getline(infile,dummy); //this is use to jump through -1 -1 -1
	}
	//close file
	infile.close();
	construct_triangle();

	return true;
}

Mesh::~Mesh(void){
	//clear used memory
	if(vertices)
		delete[] vertices;
	if(indices)
		delete[] indices;
	if(triangle_list)
		delete[] triangle_list;
}
