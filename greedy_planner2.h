#ifndef GREEDY2_H_
#define GREEDY2_H_

#include "myplanner.h"

class GreedyPlanner2 : public MyPlanner
{
	public:

		int _numcells_x;		// cells per side
		int _numcells_y;		// cells per side
		double _cellsize_x;		// size of a cell
		double _cellsize_y;		// size of a cell
		double _halfcell_x;		// size of half a cell
		double _halfcell_y;		// size of half a cell

		GreedyPlanner2(string paramfile, string logfile);
		/**
		 * Returns true if an error occurred.
		 */
		int initialize();

		Action get_action();

	private:
		Action get_action_slow();
		Action get_action_fast();
		Action get_action_faster_det();
		Action get_action_faster_eig();
		int determine_search_area();
};
#endif
