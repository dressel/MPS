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
		int bin_offset;
		vector<vector<double> > b;
		vector<vector<vector<float> > > obs_probs;

		// for right now, assume 36 bins
		DF(double l, int n);
		~DF();

		int update(Vehicle &x, double o);
		void initial_belief();
		void reset();

		void print_belief();
		void print_belief(FILE *outfile);
		void print_belief(ofstream &os);

		/**
		 * compute the mutual information
		 */
		double mutual_information(Vehicle &x, vector<double> xp);
		double mutual_information(Vehicle &x, int xv, int yv);

		void set_obs_probs(Sensor *s);

	private:
		pair<double, double> bin2deg(int obs_bin);
		pair<double,double> rel_bin_edges(double bearing, int obs_bin);

		/* Multiple dispatch O on sensor type */
		double O(double px, double py, double ph, double tx, double ty, int obs_bin, Sensor *s);
		double O(double px, double py, double tx, double ty, int obs_bin, BearingOnly *bo);
		double O(double px, double py, double ph, double tx, double ty, int obs_bin, DirOmni *s);
		
		double O(int xr, int yr, int obs_bin, Sensor *s);
		double O(int xr, int yr, int obs_bin, BearingOnly *bo);
		double O_start(int xr, int yr, int obs_bin, BearingOnly *bo);
		double O_start(int xr, int yr, int obs_bin, Sensor *bo);

		/* Multiple dispatch obs2bin on sensor type */
		int obs2bin(double o, Sensor *s);
		int obs2bin(double o, BearingOnly *s);
		int obs2bin(double o, DirOmni *s);

		/* helpers for mutual information */
		double p_obs();
		double p_obs(Vehicle &x, double xp, double yp, double hp, int ob);
		double p_obs2(Vehicle &x, int xv, int yv, int ob);

};
#endif	// FILTER_DF_H_
