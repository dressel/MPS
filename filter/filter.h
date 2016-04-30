#ifndef FILTER_FILTER_H_
#define FILTER_FILTER_H_

#include <iostream>
#include <stdio.h>
#include "../vehicle/vehicle.h"

// type 0 = bearing only
// type 1 = dir omni
class Filter
{
	public:
		int type;
		virtual int update(Vehicle x, double o) = 0;
		virtual void print_belief() = 0;
};
#endif
