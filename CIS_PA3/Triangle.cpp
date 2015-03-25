#include "Triangle.h"

Triangle::Triangle(void){}
//initialize the triangle 
Triangle::Triangle(Vector3d a,Vector3d b,Vector3d c){
	vertex[0] = a;
	vertex[1] = b;
	vertex[2] = c;
}

double Triangle::distance(const Vector3d tar){
	Vector3d closest = closest_point(tar);
	return (tar - closest).norm();
}

Vector3d Triangle::closest_point(const Vector3d a){

	Vector3d c;            // closest point on the triangle
	Vector3d p,q,r;        // coordinates of the vertices of the triangle
	p=vertex[0];
	q=vertex[1];
	r=vertex[2];

	double l,u;            // l=lambda, u= mu

	MatrixXd A(3,1);
	A= a-p;

	MatrixXd B(3,2);
	B.block<3,1>(0,0)=q-p;
	B.block<3,1>(0,1)=r-p;

	MatrixXd X(2,1);
	X= (((B.transpose())*B).inverse())*(B.transpose())*A;             // least squares approach for finding l and u

	l=X(0,0);
	u=X(1,0);

	c=p+l*(q-p)+u*(r-p);
	//find shortest ponit according to projection
	if (l >=0 && u >=0 && l+u <=1)
	{
		return c;
	}
	if(u<0)
	{
		Vector3d c1=ProjectOnSegment(c,p,q);
		return c1;
	}
	if(l<0)
	{
		Vector3d c1=ProjectOnSegment(c,r,p);
		return c1;	
	}
	if(l+u>1)
	{
		Vector3d c1=ProjectOnSegment(c,q,r);
		return c1;	
	}
	return Vector3d();

}


Vector3d Triangle::ProjectOnSegment(Vector3d c,Vector3d p,Vector3d q)
{
	double l=((c-p).dot(q-p))/((q-p).dot(q-p));
	double l1=max(0.0,min(l,1.0));

	return (p+l1*(q-p));
}