/*
We are implementing our octree structure according to the explainations and sample code
from Brandon Pelfrey.
Please reference the following website for more information
http://www.brandonpelfrey.com/blog/coding-a-simple-octree/
*/
#ifndef OctreePoint_H
#define OctreePoint_H
#include "Eigen/Core"
#include "Triangle.h"
#include <algorithm>
// Simple point data type to insert into the tree.
// Have something with more interesting behavior inherit
// from this in order to store other attributes in the tree.
class OctreePoint {
	Vector3d position;
	Triangle triangle; 
public:
	OctreePoint() { }
	OctreePoint(const Vector3d& position) : position(position) { }
	inline const Vector3d& getPosition() const { return position; }
	inline Triangle& getTriangle() { return triangle; } //we store the triangle inforamtion along with the node.
	//we store the centroid of each triangle as the postion of a node in the octree, for searching...
	//...we search the position in the tree.
	inline void setPosition() {  //set position should be put after set triangle
		position = (triangle.vertex[2] + triangle.vertex[1] + triangle.vertex[0]) / 3;
	}
	inline void setTriangle(const Triangle& t) { triangle = t; } 
};

#endif
