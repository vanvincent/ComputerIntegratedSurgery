/*
Title: CIS PA3
Date: 11.30.2014
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
#include <vector>
#include "Eigen/SVD"

#define THRESHOLD 0.001
#define ITER_MAX 1000
#define PERCENTAGE 0.8
using namespace std;
using namespace Eigen;

void init();
Vector4d findtip(Matrix4d,Matrix4d,Vector3d);
Vector3d findtip3(Matrix4d,Matrix4d,Vector3d);
Vector3d mesh_point_octree(Vector3d tip);
int generateforicp(string data_set, string data_type);
double error_analysis(string data_set, string data_type);
double error_analysis_mean(string data_set, string data_type);
double error_analysis_std(string data_set, string data_type,double mean);
void analyze_result();
void compare_result();
Matrix4d icp(Vector4d* pointcloud,Mesh& mesh,int size,Vector3d*);
//globle variables
Mesh bone;
Body bodya,bodyb;
Octree *octree;
OctreePoint *octreePoints;

int main(){
	//initalize the octree and mesh
	init();
	//generate outputs
	generateforicp("A","Debug");
	generateforicp("B","Debug");
	generateforicp("C","Debug");
	generateforicp("D","Debug");
	generateforicp("E","Debug");
	generateforicp("F","Debug");
	generateforicp("G","Unknown");
	generateforicp("H","Unknown");
	generateforicp("J","Unknown");
	generateforicp("K","Unknown");

	//analyize results
	analyze_result();
	compare_result();
	
	delete octree;
	delete octreePoints;
	
	return 0;

}
//initialization function that read mesh and build octree
void init(){
	bone.readmeshfile("Data/Problem4MeshFile.sur");
	bodya.read("Data/Problem4-BodyA.txt");
	bodyb.read("Data/Problem4-BodyB.txt");
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
// This fucntion will prepate mesh and points cloud for icp
int generateforicp(string data_set, string data_type){
	//declare of variables
	ofstream outfile;
	OPTtracker OPT;
	Registration reg;

	outfile.open (("OUTPUT/PA4-" + data_set + "-"+data_type+"-OUTPUT.txt").c_str());
	OPT.read("Data/PA4-"+data_set+"-"+data_type+"-SampleReadingsTest.txt",bodya.num_markers,bodyb.num_markers);
	outfile<<OPT.samples<<" "<<"PA4-"<<data_set<<"-"<<data_type<<"-Output.txt"<<endl;
	cout<<OPT.samples<<" "<<"PA4-"<<data_set<<"-"<<data_type<<"-Output.txt"<<endl;

	Vector4d* q= new Vector4d[OPT.samples];
	double start = stopwatch();
	//do registration to find out tip 
	for(int j=0;j<OPT.samples;j++){

		Matrix4d Fa = reg.horn(bodya.markers,OPT.data[j].BodyA,bodya.num_markers);
		Matrix4d Fb = reg.horn(bodyb.markers,OPT.data[j].BodyB,bodyb.num_markers);
		q[j] = findtip(Fa,Fb,bodya.tip);
		 
	}

	Vector3d* closest = new Vector3d[OPT.samples];
	Matrix4d F = icp(q,bone,OPT.samples,closest);
	cout<<"We find F:"<<endl<<F<<endl;
	double T = stopwatch() - start;
	printf("Operation takes %.5f sec.\n",T);
	
	for(int j=0;j<OPT.samples;j++){
		Vector4d re = F * q[j];
		outfile<< fixed;
		outfile<<setprecision(3);
		outfile<<setw(9)<<re(0)<<setw(9)<<re(1)<<setw(9)<<re(2)<<
		setw(9)<<closest[j](0)<<setw(9)<<closest[j](1)<<setw(9)<<closest[j](2)<<setw(9)<<setprecision(3)<<(re.block<3,1>(0,0)-closest[j]).norm()<<endl;
	}
	delete[] closest;
	return 0;
}

//helper function that convert the homogenuous representation into 3d representation
Vector4d findtip(Matrix4d Fa,Matrix4d Fb,Vector3d tip){
	Vector4d tip4;
	tip4(3) = 1;
	tip4.block<3,1>(0,0) = tip;
	Vector4d d = Fb.inverse() * Fa * tip4;
	return d;
}
//this is the old findtip function
Vector3d findtip3(Matrix4d Fa,Matrix4d Fb,Vector3d tip){
	Vector4d tip4;
	tip4(3) = 1;
	tip4.block<3,1>(0,0) = tip;
	Vector4d d = Fb.inverse() * Fa * tip4;
	return d.block<3,1>(0,0);
}

//An octree optimization method that reduce the search time
Vector3d mesh_point_octree(Vector3d tip){

	Vector3d qmin, qmax; //maximum and minimum searching bound around the tip
	// Create a very small query box, and search the octree
	double bound = 1;
	std::vector<OctreePoint*> results; //a set of nodes that lay in the bounding box
	//if the bounding box is too small, increase the bound and search again
	while(results.size() == 0){
		qmin = tip + Vector3d(-(5.+bound),-(5.+bound),-(5.+bound));
		qmax = tip + Vector3d(5.+bound,5.+bound,5.+bound);
		octree->getPointsInsideBox(qmin, qmax, results);
		bound+=1;

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
	infile.open (("OUTPUT/PA4-" + data_set + "-"+data_type+"-OUTPUT.txt").c_str());
	infile2.open (("Data/PA4-" + data_set + "-"+data_type+"-Answer.txt").c_str());
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
//this compute the mean for each set
double error_analysis_mean(string data_set, string data_type){
	

	ifstream infile,infile2;
	infile.open (("OUTPUT/PA4-" + data_set + "-"+data_type+"-OUTPUT.txt").c_str());

	if(!infile.is_open()){
		cout<<"Error openning file."<<endl;
		return false;
	}


	double error = 0;
	string dummy;
	int samples;
	infile>>samples;
	getline(infile,dummy); 
	//looping to read all infomation
	for(int i=0;i<samples;i++){
		double dx,dy;
		infile>>dx>>dx>>dx>>dx>>dx>>dx>>dy; //reading
		error +=abs(dy); //compute the absolute differences
	}


	//close file
	infile.close();
	return error/samples; //output average error
}
//this compute the standard deviation for each set
double error_analysis_std(string data_set, string data_type,double mean){
	

	ifstream infile,infile2;
	infile.open (("OUTPUT/PA4-" + data_set + "-"+data_type+"-OUTPUT.txt").c_str());

	if(!infile.is_open()){
		cout<<"Error openning file."<<endl;
		return false;
	}


	double std = 0;
	string dummy;
	int samples;
	infile>>samples;
	getline(infile,dummy); 
	//looping to read all infomation
	for(int i=0;i<samples;i++){
		double dx,dy;
		infile>>dx>>dx>>dx>>dx>>dx>>dx>>dy; //reading
		std += pow((abs(dy) - mean),2); //compute the absolute differences
	}

	//close file
	infile.close();
	return sqrt(std/samples); //output average error
}
//this will compare our results to the debug outputs
void compare_result(){
	ofstream outfile;
	outfile.open ("OUTPUT/PA4-error_comparison.txt");
	outfile<<"Average error for set A between our outputs and the debug outputs:"<<error_analysis("A","Debug")<<endl;
	outfile<<"Average error for set B between our outputs and the debug outputs:"<<error_analysis("B","Debug")<<endl;
	outfile<<"Average error for set C between our outputs and the debug outputs:"<<error_analysis("C","Debug")<<endl;
	outfile<<"Average error for set D between our outputs and the debug outputs:"<<error_analysis("D","Debug")<<endl;
	outfile<<"Average error for set E between our outputs and the debug outputs:"<<error_analysis("E","Debug")<<endl;
	outfile<<"Average error for set F between our outputs and the debug outputs:"<<error_analysis("F","Debug")<<endl;
	outfile.close();
}
//this will generate an output table to show mean and std for all set
void analyze_result(){

	ofstream outfile;
	outfile.open ("OUTPUT/PA4-error_analysis.txt");
	outfile<<"               mean            std"<<endl;
	double mean,std;
	outfile<<fixed;
	outfile<<setprecision(7);
	mean = error_analysis_mean("A","Debug");
	std  = error_analysis_std("A","Debug",mean);
	outfile<<"Set A:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("B","Debug");
	std  = error_analysis_std("B","Debug",mean);
	outfile<<"Set B:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("C","Debug");
	std  = error_analysis_std("C","Debug",mean);
	outfile<<"Set C:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("D","Debug");
	std  = error_analysis_std("D","Debug",mean);
	outfile<<"Set D:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("E","Debug");
	std  = error_analysis_std("E","Debug",mean);
	outfile<<"Set E:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("F","Debug");
	std  = error_analysis_std("F","Debug",mean);
	outfile<<"Set F:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("G","Unknown");
	std  = error_analysis_std("G","Unknown",mean);
	outfile<<"Set G:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("H","Unknown");
	std  = error_analysis_std("H","Unknown",mean);
	outfile<<"Set H:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("J","Unknown");
	std  = error_analysis_std("J","Unknown",mean);
	outfile<<"Set J:"<<setw(15)<<mean<<setw(15)<<std<<endl;
	mean = error_analysis_mean("K","Unknown");
	std  = error_analysis_std("K","Unknown",mean);
	outfile<<"Set K:"<<setw(15)<<mean<<setw(15)<<std<<endl;

	outfile.close();
}

//This is the icp algorithm
Matrix4d icp(Vector4d* pointcloud,Mesh& mesh,int size,Vector3d* c){
	
	Registration reg;
	Matrix4d F = Matrix4d::Identity(); //initial guess
	int sample_size = size;
	double dis;
	double eta = 100; //initialize eta to be a large value
	double epsilon = 0;
	double epsilon_old; 
	int iter= 0 ;
	double err_max = 0;
	vector<Vector3d> A;
	vector<Vector3d> B;
	//iteration
	while(1){

		iter++;
		cout<<fixed;
		cout<<setprecision(7);
		cout<<"iteration:"<<setw(5)<<iter<<"  epsilon:"<<setw(11)<<epsilon<<"  matching pairs:"<<setw(5)<<sample_size<<"  eta:"<<setw(11)<<eta<<"  maximun Error:"<<setw(11)<<err_max<<endl;
		epsilon_old = epsilon;
		epsilon = 0;
		err_max = INT_MIN;

		
		//genrate A and B, subset of valid pairs
		for(int i = 0; i< size;i++){
			c[i] = mesh_point_octree((F * pointcloud[i]).block<3,1>(0,0));
			dis = (c[i] - (F * pointcloud[i]).block<3,1>(0,0)).norm();
			if(dis < eta){
				A.push_back(pointcloud[i].block<3,1>(0,0));
				B.push_back(c[i]);
			}
		}

		sample_size = A.size();

		Vector3d* a = new Vector3d[sample_size];
		Vector3d* b = new Vector3d[sample_size];
		for(int i = 0;i< sample_size;i++){
			a[i] = A[i];
			b[i] = B[i];
		}
		// do registration among the two subset
		Matrix4d F_new = reg.horn(a,b,sample_size);
		double* err= new double[sample_size];

		//calculate the residuals
		for(int i = 0;i< sample_size;i++){
			err[i] = (b[i] - F_new.block<3,3>(0,0) * a[i] - F_new.block<3,1>(0,3)).norm();
			if(err[i] > err_max)
					err_max = err[i];
			epsilon += err[i];
		}
		epsilon/=sample_size;
		//converge condition
		if(abs(epsilon - epsilon_old) <= 0.0000001){
			//adjust eta
			eta = 0.95 * err_max;
			//termination condidtion
			if(epsilon < THRESHOLD || iter > ITER_MAX || sample_size < PERCENTAGE * size){ 
				delete[] a;
				delete[] b;
				return F_new;
			}
			
		}

		F = F_new; //update transformation
		A.clear();
		B.clear();
		delete[] a;
		delete[] b;
	}
}










