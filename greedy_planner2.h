#ifndef GREEDY2_H_
#define GREEDY2_H_

#include "myplanner.h"

class GreedyPlanner2 : public MyPlanner
{
	public:

		int _n;
		double _cell_size;

		GreedyPlanner2(string paramfile, string logfile);
		/**
		 * Returns true if an error occurred.
		 */
		int initialize();

		Action get_action();

	private:
		Action get_action_slow();
		Action get_action_fast();
};
#endif
