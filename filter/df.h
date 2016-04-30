#ifndef FILTER_DF_H_
#define FILTER_DF_H_

#include <vector>
#include "../mps_math.h"
#include "filter.h"

using std::vector;
using std::pair;

class DF : public Filter
{
	public:
		double l;
		int n;
		double cell_size;
		int num_bins;
		vector<vector<double> > b;

		// for right now, assume 36 bins
		DF(double l, int n);
		int update(Vehicle x, double o);
		double mutual_information(Vehicle x, vector<double> xp);
		double p_obs();
		double p_obs(Vehicle x, double xp, double yp, double hp, int ob);
		int obs2bin(double o, Sensor *s);
		int obs2bin(double o, BearingOnly *s);
		void print_belief();
		void initial_belief();
		void reset();
		double O(double px, double py, double ph, double tx, double ty, int obs_bin, Sensor *s);
		double O(double px, double py, double tx, double ty, int obs_bin, BearingOnly *bo);
		// TODO move this guy, he shouldn't be here
		double true_bearing(double px, double py, double tx, double ty);
		pair<double,double> rel_bin_edges(double bearing, int obs_bin);
		double fit_180(double angle);
		pair<double, double> bin2deg(int obs_bin);

};
#endif	// FILTER_DF_H_
