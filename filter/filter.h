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
		virtual ~Filter() {};
		virtual int update(Vehicle &x, double o) = 0;

		/**
		 * 0 = DF
		 * 1 = PF
		 */
		int type;

		/* For printing */
		virtual void print_belief() = 0;
		virtual void print_belief(FILE *outfile) = 0;
		virtual void print_belief(ofstream &os) = 0;

		/* Computing mutual information */
		virtual double mutual_information(Vehicle &x, vector<double> xp) =0;
		virtual double mutual_information(Vehicle &x, int xr, int yr) = 0;

		/* Computing some basic statistics */
		virtual std::pair<double,double> centroid() = 0;
		virtual vector<double> covariance() = 0;
		virtual vector<double> meancov() = 0;
};
#endif
