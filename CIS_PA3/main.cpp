/*
Title: CIS PA3
Author: Han Xiao & Kunal Saluja
*/
#include "Mesh.h"
#include "Triangle.h"
#include "Body.h"
#include "OPTtracker.h"
#include "Registration.h"
#include "OctreePoint.h"
#include "Octree.h"
#include "Stopwatch.h"
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <climits> 
#include <iomanip>

using namespace std;
using namespace Eigen;

Vector3d findtip(Matrix4d,Matrix4d,Vector3d);
Vector3d mesh_point_bruteforce(Vector3d tip);
Vector3d mesh_point_octree(Vector3d tip);
int generatedata(string,string);
double error_analysis(string data_set, string data_type);
void init();
//globle variables
Mesh bone;
Body bodya,bodyb;
Octree *octree;
OctreePoint *octreePoints;

int main(){

	init();
	generatedata("A","Debug");
	generatedata("B","Debug");
	generatedata("C","Debug");
	generatedata("D","Debug");
	generatedata("E","Debug");
	generatedata("F","Debug");
	generatedata("G","Unknown");
	generatedata("H","Unknown");
	cout<<"Average error for set A:"<<error_analysis("A","Debug")<<endl;
	cout<<"Average error for set B:"<<error_analysis("B","Debug")<<endl;
	cout<<"Average error for set C:"<<error_analysis("C","Debug")<<endl;
	cout<<"Average error for set D:"<<error_analysis("D","Debug")<<endl;
	cout<<"Average error for set E:"<<error_analysis("E","Debug")<<endl;
	cout<<"Average error for set F:"<<error_analysis("F","Debug")<<endl;
	delete octree;
	delete octreePoints;

	return 0;

}
//initialization function that read mesh and build octree
void init(){
	bone.readmeshfile("Data/Problem3Mesh.sur");
	bodya.read("Data/Problem3-BodyA.txt");
	bodyb.read("Data/Problem3-BodyB.txt");
	//declare an octree and octree nodes
	// Create a new Octree centered at the origin with halfwidth 200
	octree = new Octree(Vector3d(0,0,0), Vector3d(200,200,200));
	// Insert the points into the octree
	octreePoints = new OctreePoint[bone.num_triangles];

	for(int i=0; i<bone.num_triangles; ++i) {
		octreePoints[i].setTriangle(bone.triangle_list[i]); //insert triangle into node
		octreePoints[i].setPosition(); //settign the posiiton
		octree->insert(octreePoints + i); //insert node into octree
	}
}
//function that read specific inputs, do registration and then find closest point on mesh
int generatedata(string data_set, string data_type){
	//declare of variables
	ofstream outfile;
	OPTtracker OPT;
	Registration reg;

	outfile.open (("OUTPUT/PA3-" + data_set + "-"+data_type+"-OUTPUT.txt").c_str());
	OPT.read("Data/PA3-"+data_set+"-"+data_type+"-SampleReadingsTest.txt",bodya.num_markers,bodyb.num_markers);
	outfile<<OPT.samples<<" "<<"PA3-"<<data_set<<"-"<<data_type<<"-Output.txt"<<endl;

	double start = stopwatch();
	//do registration to find out tip 
	for(int j=0;j<OPT.samples;j++){

		Matrix4d Fa = reg.horn(bodya.markers,OPT.data[j].BodyA,bodya.num_markers);
		Matrix4d Fb = reg.horn(bodyb.markers,OPT.data[j].BodyB,bodyb.num_markers);
		Vector3d tip = findtip(Fa,Fb,bodya.tip);	
		//search and output result
		Vector3d re =mesh_point_octree(tip);
		outfile<< fixed;
		outfile<<setprecision(2);
		outfile<<setw(9)<<tip(0)<<setw(9)<<tip(1)<<setw(9)<<tip(2)<<
		setw(9)<<re(0)<<setw(9)<<re(1)<<setw(9)<<re(2)<<setw(9)<<setprecision(3)<<(tip-re).norm()<<endl;
		
	}
	
	double T = stopwatch() - start;
	printf("Octree search takes %.5f sec.\n",T);
	/*  The timing analysis part, we compared the run time between bruteforce and octree method
	start = stopwatch();
	for(int j=0;j<OPT.samples;j++){

		Matrix4d Fa = reg.horn(bodya.markers,OPT.data[j].BodyA,bodya.num_markers);
		Matrix4d Fb = reg.horn(bodyb.markers,OPT.data[j].BodyB,bodyb.num_markers);
		Vector3d tip = findtip(Fa,Fb,bodya.tip);	
		//search and output result
		Vector3d re =mesh_point_bruteforce(tip);
		
	}
	T = stopwatch() - start;
	printf("Bruteforce search takes %.5f sec.\n",T);
	*/
	return 0;
}

//helper function that convert the homogenuous representation into 3d representation
Vector3d findtip(Matrix4d Fa,Matrix4d Fb,Vector3d tip){
	Vector4d tip4;
	tip4(3) = 1;
	tip4.block<3,1>(0,0) = tip;
	Vector4d d = Fb.inverse() * Fa * tip4;
	return d.block<3,1>(0,0);
}

//A bruteforce method that search all meshes. we are not using this method in PA3
Vector3d mesh_point_bruteforce(Vector3d tip){
	double min = INT_MAX;
	Vector3d c;
	for(int i=0;i<bone.num_triangles;i++){
		double dis = bone.triangle_list[i].distance(tip);
		if(dis < min){
			min = dis;
			c = bone.triangle_list[i].closest_point(tip);
		}
	}
	return c;
}

//An octree optimization method that reduce the search time
Vector3d mesh_point_octree(Vector3d tip){

	Vector3d qmin, qmax; //maximum and minimum searching bound around the tip
	// Create a very small query box, and search the octree
	double bound = 0;
	std::vector<OctreePoint*> results; //a set of nodes that lay in the bounding box
	//if the bounding box is too small, increase the bound and search again
	while(results.size() == 0){
		qmin = tip + Vector3d(-(5.+bound),-(5.+bound),-(5.+bound));
		qmax = tip + Vector3d(5.+bound,5.+bound,5.+bound);
		octree->getPointsInsideBox(qmin, qmax, results);
		bound+=0.5;
	}
	//find the minimum distance in among the results
	double min = INT_MAX;
	Vector3d c;
	for(int i=0;i<results.size();i++){
		double dis = results[i]->getTriangle().distance(tip);
		if(dis < min){
			min = dis;
			c = results[i]->getTriangle().closest_point(tip);
		}
	}

	return c;
}
// This function will analysis the error between our outputs and the provided debug outputs
double error_analysis(string data_set, string data_type){
	
	ifstream infile,infile2;
	infile.open (("OUTPUT/PA3-" + data_set + "-"+data_type+"-OUTPUT.txt").c_str());
	infile2.open (("Data/PA3-" + data_set + "-"+data_type+"-Output.txt").c_str());
	if(!infile.is_open()){
		cout<<"Error openning file."<<endl;
		return false;
	}

	if(!infile2.is_open()){
		cout<<"Error openning file2."<<endl;
		return false;
	}

	double error = 0;
	string dummy;
	int samples;
	infile>>samples;
	getline(infile,dummy); 
	infile2>>samples;
	getline(infile2,dummy); 
	//looping to read all infomation
	for(int i=0;i<samples;i++){
		double dx,dy,dy2;
		infile>>dx>>dx>>dx>>dx>>dx>>dx>>dy; //reading
		infile2>>dx>>dx>>dx>>dx>>dx>>dx>>dy2; //reading
		error +=abs(dy-dy2); //compute the absolute differences
	}

	//close file
	infile.close();
	infile2.close();
	return error/samples; //output average error
}















