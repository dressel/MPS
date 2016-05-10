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

		// TODO: function that prints out actions so we can see if correct

	//private:
		vector<vector<float> > actions;
		vector<float> get_action();
		int bo_initialize();
		int do_initialize();
};
#endif
