#ifndef GREEDY_H_
#define GREEDY_H_

#include "myplanner.h"

class GreedyPlanner : public MyPlanner
{
	public:

		/**
		 * Constructor
		 */
		GreedyPlanner(string paramfile, string logfile);

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

	//private:
		vector<vector<float> > actions;
		vector<float> find_best_action();
		int bo_initialize();
		int do_initialize();
};
#endif
