#ifndef OPTICALTRACKER_INCLUDED
#define OPTICALTRACKER_INCLUDED
#include <stdio.h>
#include <Eigen/Core>
#include "Marker.h"
#include <string>
using namespace Eigen;
using namespace std;
class OpticalTracker {
public:
	OpticalTracker(void);
	int frame_size;

	struct Reading{
	public:			
		int emled_size;
		int caliobjled_size;
		Marker *emled;
		Marker *caliobjled;
	};

	Reading* data;
	int read(string);
	int readem(string);
	void write(char *);
	~OpticalTracker();
};

#endif