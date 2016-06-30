#ifndef FILTER_FILTER_H_
#define FILTER_FILTER_H_

#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include "../vehicle/vehicle.h"

using std::ofstream;

// type 0 = bearing only
// type 1 = dir omni
class Filter
{
	public:
		int type;
		virtual ~Filter() {};
		virtual int update(Vehicle &x, double o) = 0;
		virtual void print_belief() = 0;
		virtual void print_belief(FILE *outfile) = 0;
		virtual void print_belief(ofstream &os) = 0;
		virtual double mutual_information(Vehicle &x, vector<double> xp) = 0;
};
#endif
