#ifndef GREEDY2_H_
#define GREEDY2_H_

#include "myplanner.h"

class GreedyPlanner2 : public MyPlanner
{
	public:

		/**
		 * Constructor
		 */
		GreedyPlanner2(string paramfile, string logfile);

		/**
		 * Required function to direct vehicle.
		 *
		 * Returns a 3-element vector of floats: (d_north, d_east, d_yaw).
		 */
		vector<float> action();

		/**
		 * Returns true if an error occurred.
		 */
		int initialize();

		// TODO: function that prints out actions so we can see if correct

		vector<float> find_best_action();
		int _n;
		double _cell_size;
};
#endif
