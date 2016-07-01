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
		 * Returns true if an error occurred.
		 */
		int initialize();

		Action get_action();

	private:
		vector<vector<float> > actions;
		int bo_initialize();
		int do_initialize();
};
#endif
