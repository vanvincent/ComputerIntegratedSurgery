#ifndef OPTICALTRACKER_INCLUDED
#define OPTICALTRACKER_INCLUDED
#include <stdio.h>
#include <Eigen/Core>
#include "Marker.h"
using namespace Eigen;
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
	int read(char*);
	int readem(char*);
	void write(char *);
	~OpticalTracker();
};

#endif