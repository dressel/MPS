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
		 * Returns true if an error occurred.
		 */
		int initialize();

		// TODO: function that prints out actions so we can see if correct

		vector<float> get_action();
		vector<float> get_action_slow();
		vector<float> get_action_fast();
		int _n;
		double _cell_size;
};
#endif
